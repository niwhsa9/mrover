#include "simulator.hpp"

namespace mrover {

    using namespace std::literals;

    constexpr int SDL_OK = 0;

    constexpr float DEG2RAD = std::numbers::pi_v<float> / 180.0f;

    static std::string const CUBE_PRIMITIVE_URI = "package://mrover/urdf/meshes/cube.glb";
    static std::string const SPHERE_PRIMITIVE_URI = "package://mrover/urdf/meshes/sphere.glb";
    static std::string const CYLINDER_PRIMITIVE_URI = "package://mrover/urdf/meshes/cylinder.glb";

    static auto check(bool condition) -> void {
        if (!condition) throw std::runtime_error(std::format("SDL Error: {}", SDL_GetError()));
    }

    static auto btTransformToSe3(btTransform const& transform) -> SE3 {
        btVector3 translation = transform.getOrigin();
        btQuaternion rotation = transform.getRotation();
        return SE3{R3{translation.x(), translation.y(), translation.z()}, SO3{rotation.w(), rotation.x(), rotation.y(), rotation.z()}};
    }

    auto perspective(float fovY, float aspect, float zNear, float zFar) -> Eigen::Matrix4f {
        // Equivalent to glm::perspectiveRH_NO
        float theta = fovY * 0.5f;
        float range = zFar - zNear;
        float invtan = 1.0f / tanf(theta);
        Eigen::Matrix4f result;
        result << invtan / aspect, 0.0f, 0.0f, 0.0f,
                0.0f, invtan, 0.0f, 0.0f,
                0.0f, 0.0f, -(zFar + zNear) / range, -2.0f * zFar * zNear / range,
                0.0f, 0.0f, -1.0f, 0.0f;
        return result;
    }

    auto SimulatorNodelet::initRender() -> void {
        // Force laptops with NVIDIA GPUs to use it instead of the integrated graphics
        setenv("DRI_PRIME", "1", true);
        setenv("__NV_PRIME_RENDER_OFFLOAD", "1", true);
        setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", true);

        check(SDL_Init(SDL_INIT_VIDEO) == SDL_OK);
        NODELET_INFO_STREAM(std::format("Initialized SDL Version: {}.{}.{}", SDL_MAJOR_VERSION, SDL_MINOR_VERSION, SDL_PATCHLEVEL));

        SDL_DisplayMode displayMode;
        check(SDL_GetDesktopDisplayMode(0, &displayMode) == SDL_OK);
        assert(displayMode.w > 0 && displayMode.h > 0);

        auto w = static_cast<int>(displayMode.w * 0.8), h = static_cast<int>(displayMode.h * 0.8);
        mWindow = SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow>{"MRover Simulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w, h, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI};
        NODELET_INFO_STREAM(std::format("Created window of size: {}x{}", w, h));

        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24) == SDL_OK);
        check(SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8) == SDL_OK);
        mGlContext = SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext>{mWindow.get()};
        check(SDL_GL_SetSwapInterval(0) == SDL_OK);
        NODELET_INFO_STREAM(std::format("Initialized OpenGL Version: {}", reinterpret_cast<char const*>(glGetString(GL_VERSION))));
        NODELET_INFO_STREAM(std::format("\tShading Language Version: {}", reinterpret_cast<char const*>(glGetString(GL_SHADING_LANGUAGE_VERSION))));
        NODELET_INFO_STREAM(std::format("\tVendor: {}", reinterpret_cast<char const*>(glGetString(GL_VENDOR))));
        NODELET_INFO_STREAM(std::format("\tRenderer: {}", reinterpret_cast<char const*>(glGetString(GL_RENDERER))));

        glewExperimental = GL_TRUE;
        check(glewInit() == GLEW_OK);

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glDepthFunc(GL_LESS);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        float ddpi, hdpi, vdpi;
        SDL_GetDisplayDPI(0, &ddpi, &hdpi, &vdpi);
        ImGuiIO& io = ImGui::GetIO();
        ImGuiStyle& style = ImGui::GetStyle();
        io.FontGlobalScale = ddpi / 128.0f;
        style.ScaleAllSizes(ddpi / 128.0f);
        ImGui_ImplSDL2_InitForOpenGL(mWindow.get(), mGlContext.get());
        ImGui_ImplOpenGL3_Init("#version 410 core");

        auto shadersPath = std::filesystem::path{std::source_location::current().file_name()}.parent_path() / "shaders";
        mShaderProgram = {
                Shader{shadersPath / "pbr.vert", GL_VERTEX_SHADER},
                Shader{shadersPath / "pbr.frag", GL_FRAGMENT_SHADER}};

        mUriToMesh.emplace(CUBE_PRIMITIVE_URI, CUBE_PRIMITIVE_URI);
        mUriToMesh.emplace(SPHERE_PRIMITIVE_URI, SPHERE_PRIMITIVE_URI);
        mUriToMesh.emplace(CYLINDER_PRIMITIVE_URI, CYLINDER_PRIMITIVE_URI);
    }

    auto SimulatorNodelet::renderMesh(Mesh const& mesh, Eigen::Matrix4f const& modelToWorld) -> void {
        GLint modelToWorldId = glGetUniformLocation(mShaderProgram.handle, "modelToWorld");
        assert(modelToWorldId != GL_INVALID_INDEX);

        glUniform(modelToWorldId, modelToWorld);

        for (auto const& [vao, _vbo, _ebo, indicesCount]: mesh.bindings) {
            assert(vao != GL_INVALID_HANDLE);
            assert(indicesCount > 0);

            glBindVertexArray(vao);
            glDrawElements(GL_TRIANGLES, indicesCount, GL_UNSIGNED_INT, nullptr);
        }
    }

    auto SimulatorNodelet::renderUrdf(URDF const& urdf) -> void {
        assert(mShaderProgram.handle != GL_INVALID_HANDLE);

        auto traverse = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
            assert(mShaderProgram.handle != GL_INVALID_HANDLE);
            glUseProgram(mShaderProgram.handle);

            if (link->visual && link->visual->geometry) {
                if (auto urdfMesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry)) {
                    Mesh const& mesh = mUriToMesh.at(urdfMesh->filename);
                    renderMesh(mesh, SE3{}.matrix().cast<float>());
                }
            }
            for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
                self(self, urdf.model.getLink(child_joint->child_link_name));
            }
        };
        traverse(traverse, urdf.model.getRoot());
    }

    auto SimulatorNodelet::renderUpdate() -> void {
        int w{}, h{};
        SDL_GetWindowSize(mWindow.get(), &w, &h);

        glViewport(0, 0, w, h);
        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        GLint worldToCameraId = glGetUniformLocation(mShaderProgram.handle, "worldToCamera");
        assert(worldToCameraId != GL_INVALID_INDEX);

        // Convert from ROS's right-handed +x forward, +y left, +z up to OpenGL's right-handed +x right, +y up, +z backward
        Eigen::Matrix4f rosToGl;
        rosToGl << 0, -1, 0, 0, // OpenGL x = -ROS y
                0, 0, 1, 0,     // OpenGL y = ROS z
                -1, 0, 0, 0,    //  OpenGL z = -ROS x
                0, 0, 0, 1;
        Eigen::Matrix4f worldToCamera = rosToGl * mCameraInWorld.matrix().inverse().cast<float>();
        glUniform(worldToCameraId, worldToCamera);

        GLint cameraToClipId = glGetUniformLocation(mShaderProgram.handle, "cameraToClip");
        assert(cameraToClipId != GL_INVALID_INDEX);

        float aspect = static_cast<float>(w) / static_cast<float>(h);
        Eigen::Matrix4f cameraToClip = perspective(60.0f * DEG2RAD, aspect, 0.1f, 100.0f).cast<float>();
        glUniform(cameraToClipId, cameraToClip);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        for (URDF const& urdf: mUrdfs) {
            renderUrdf(urdf);
        }

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        for (auto const& collisionObject: mCollisionObjects) {
            btTransform rbInWorld = collisionObject->getWorldTransform();
            btCollisionShape const* shape = collisionObject->getCollisionShape();
            Eigen::Matrix4f modelInWorld = btTransformToSe3(rbInWorld).matrix().cast<float>();
            if (auto* box = dynamic_cast<btBoxShape const*>(shape)) {
                btVector3 extents = box->getHalfExtentsWithoutMargin();
                modelInWorld.block<3, 3>(0, 0) *= Eigen::DiagonalMatrix<float, 3>{extents.x(), extents.y(), extents.z()};
                renderMesh(mUriToMesh.at(CUBE_PRIMITIVE_URI), modelInWorld);
            } else if (auto* sphere = dynamic_cast<btSphereShape const*>(shape)) {
                btScalar radius = sphere->getRadius();
                modelInWorld.block<3, 3>(0, 0) *= Eigen::DiagonalMatrix<float, 3>{radius, radius, radius};
                renderMesh(mUriToMesh.at(SPHERE_PRIMITIVE_URI), modelInWorld);
            } else if (auto* cylinder = dynamic_cast<btCylinderShape const*>(shape)) {
                btVector3 extents = cylinder->getHalfExtentsWithoutMargin();
                modelInWorld.block<3, 3>(0, 0) *= Eigen::DiagonalMatrix<float, 3>{extents.x(), extents.y(), extents.z()};
                renderMesh(mUriToMesh.at(CYLINDER_PRIMITIVE_URI), modelInWorld);
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(mWindow.get());
        ImGui::NewFrame();

        // Draw fps text
        ImGui::SetNextWindowPos(ImVec2(8, 8));
        ImGui::SetNextWindowSize(ImVec2(0, 0));
        ImGui::Begin("FPS", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        ImGui::End();

        bool show_demo_window = true;
        ImGui::ShowDemoWindow(&show_demo_window);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(mWindow.get());
    }

} // namespace mrover
