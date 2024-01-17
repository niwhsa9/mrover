#pragma once

#include "pch.hpp"

#include "wgpu_objects.hpp"
#include "glfw_pointer.hpp"

using namespace std::literals;

namespace mrover {

    using Clock = std::chrono::high_resolution_clock;

    constexpr static float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;

    // Convert from ROS's right-handed +x forward, +y left, +z up to WGPU's left-handed +x right, +y up, +z backward
    static auto const ROS_TO_WGPU = (Eigen::Matrix4f{} << 0, -1, 0, 0, // WGPU x = -ROS y
                                     0, 0, 1, 0,                       // WGPU y = +ROS z
                                     1, 0, 0, 0,                       // WGPU z = +ROS x
                                     0, 0, 0, 1)
                                            .finished();

    static auto const COLOR_FORMAT = wgpu::TextureFormat::BGRA8Unorm;
    static auto const DEPTH_FORMAT = wgpu::TextureFormat::Depth32Float;

    struct Camera;
    class SimulatorNodelet;

    // Eigen stores matrices in column-major which is the same as WGPU
    // As such there is no need to modify data before uploading to the device

    struct ModelUniforms {
        Eigen::Matrix4f modelToWorld{};
        Eigen::Matrix4f modelToWorldForNormals{};

        std::uint32_t material{};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct SceneUniforms {
        Eigen::Matrix4f worldToCamera{};
        Eigen::Matrix4f cameraToClip{};

        Eigen::Vector4f lightInWorld{};
        Eigen::Vector4f cameraInWorld{};
        Eigen::Vector4f lightColor{};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct ComputeUniforms {
        Eigen::Matrix4f clipToCamera{};
        Eigen::Vector2i resolution{};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Model {
        struct Mesh {
            SharedBuffer<Eigen::Vector3f> vertices;
            SharedBuffer<Eigen::Vector3f> normals;
            SharedBuffer<Eigen::Vector2f> uvs;
            SharedBuffer<std::uint32_t> indices;
            MeshTexture texture;
        };

        // DO NOT access the mesh unless you are certain it has been set from the async loader
        std::vector<Mesh> meshes;
        // https://en.cppreference.com/w/cpp/thread/future
        boost::future<decltype(meshes)> asyncMeshesLoader;

        explicit Model(std::string_view uri);

        auto waitMeshes() -> void;

        [[nodiscard]] auto areMeshesReady() -> bool;

        ~Model();
    };

    struct URDF {
        std::string name;
        urdf::Model model;
        btMultiBody* physics = nullptr;
        std::unordered_map<std::string, int> linkNameToIndex;
        std::unordered_map<std::string, Uniform<ModelUniforms>> linkNameToUniform;

        URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init);

        auto makeCollisionShapeForLink(SimulatorNodelet& simulator, urdf::LinkConstSharedPtr const& link) -> btCollisionShape*;

        auto makeCameraForLink(SimulatorNodelet& simulator, btMultibodyLink const* link) -> Camera;

        [[nodiscard]] auto linkInWorld(std::string const& linkName) const -> SE3;
    };

    struct PeriodicTask {
        Clock::duration period{};
        Clock::time_point lastUpdate = Clock::now();

        PeriodicTask() = default;

        explicit PeriodicTask(Clock::duration period)
            : period{period} {}

        explicit PeriodicTask(float rate)
            : PeriodicTask{std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>{1.0f / rate})} {}

        [[nodiscard]] auto shouldUpdate() -> bool {
            if (Clock::time_point now = Clock::now(); now - lastUpdate > period) {
                lastUpdate = now;
                return true;
            }

            return false;
        }
    };

    struct Camera {
        btMultibodyLink const* link;
        Eigen::Vector2i resolution;
        PeriodicTask updateTask;
        ros::Publisher pcPub;

        wgpu::Texture colorTexture = nullptr;
        wgpu::TextureView colorTextureView = nullptr;
        wgpu::Texture depthTexture = nullptr;
        wgpu::TextureView depthTextureView = nullptr;
        wgpu::Buffer pointCloudBuffer = nullptr;
        wgpu::Buffer pointCloudBufferStaging = nullptr;

        Uniform<SceneUniforms> sceneUniforms{};
        wgpu::BindGroup sceneBindGroup = nullptr;
        Uniform<ComputeUniforms> computeUniforms{};
        wgpu::BindGroup computeBindGroup = nullptr;

        std::unique_ptr<wgpu::BufferMapCallback> callback = nullptr;
        bool needsMap = false;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class SimulatorNodelet final : public nodelet::Nodelet {
        friend Model;
        friend URDF;

        // Settings

        float mTargetUpdateRate = 180;
        bool mHeadless = false;
        int mQuitKey = GLFW_KEY_Q;
        int mInGuiKey = GLFW_KEY_ESCAPE;
        int mCamRightKey = GLFW_KEY_D;
        int mCamLeftKey = GLFW_KEY_A;
        int mCamForwardKey = GLFW_KEY_W;
        int mCamBackwardKey = GLFW_KEY_S;
        int mCamUpKey = GLFW_KEY_SPACE;
        int mCamDownKey = GLFW_KEY_LEFT_CONTROL;
        int mRoverRightKey = GLFW_KEY_L;
        int mRoverLeftKey = GLFW_KEY_J;
        int mRoverForwardKey = GLFW_KEY_I;
        int mRoverBackwardKey = GLFW_KEY_COMMA;
        int mRoverStopKey = GLFW_KEY_K;
        int mTogglePhysicsKey = GLFW_KEY_P;
        int mToggleRenderModelsKey = GLFW_KEY_M;
        int mToggleRenderWireframeCollidersKey = GLFW_KEY_C;

        float mFlySpeed = 5.0f;
        float mRoverLinearSpeed = 1.0f;
        float mRoverAngularSpeed = 0.5f;
        float mLookSense = 0.004f;
        float mFovDegrees = 60.0f;
        btVector3 mGravityAcceleration{0.0f, 0.0f, -9.81f};
        bool mEnablePhysics = false;
        bool mRenderModels = true;
        bool mRenderWireframeColliders = false;

        float mFloat = 0.0f;

        // ROS

        ros::NodeHandle mNh, mPnh;

        ros::Subscriber mTwistSub, mJointPositionsSub;

        ros::Publisher mPosePub;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

        Eigen::Vector3f mIkTarget{0.5, 0.1, 0.5};
        ros::Publisher mIkTargetPub;

        // Rendering

        GlfwInstance mGlfwInstance;
        GlfwPointer<GLFWwindow, glfwCreateWindow, glfwDestroyWindow> mWindow;
        wgpu::Instance mWgpuInstance = nullptr;
        wgpu::Surface mSurface = nullptr;
        wgpu::Adapter mAdapter = nullptr;
        wgpu::Device mDevice = nullptr;
        std::unique_ptr<wgpu::ErrorCallback> mErrorCallback;
        wgpu::Queue mQueue = nullptr;
        wgpu::SwapChain mSwapChain = nullptr;
        wgpu::Texture mDepthTexture = nullptr;
        wgpu::TextureView mDepthTextureView = nullptr;

        wgpu::ShaderModule mShaderModule = nullptr;
        wgpu::RenderPipeline mPbrPipeline = nullptr;

        wgpu::ComputePipeline mPointCloudPipeline = nullptr;

        std::unordered_map<std::string, Model> mUriToModel;

        bool mHasFocus = false;
        bool mInGui = false;

        Uniform<SceneUniforms> mSceneUniforms;

        // Physics

        std::unique_ptr<btDefaultCollisionConfiguration> mCollisionConfig;
        std::unique_ptr<btCollisionDispatcher> mDispatcher;
        std::unique_ptr<btHashedOverlappingPairCache> mOverlappingPairCache;
        std::unique_ptr<btDbvtBroadphase> mBroadphase;
        std::unique_ptr<btMultiBodyConstraintSolver> mSolver;
        std::unique_ptr<btMultiBodyDynamicsWorld> mDynamicsWorld;
        std::vector<std::unique_ptr<btCollisionShape>> mCollisionShapes;
        std::vector<std::unique_ptr<btMultiBody>> mMultiBodies;
        std::vector<std::unique_ptr<btMultiBodyLinkCollider>> mMultibodyCollider;
        std::vector<std::unique_ptr<btMultiBodyConstraint>> mMultibodyConstraints;

        std::unordered_map<btBvhTriangleMeshShape*, std::string> mMeshToUri;

        struct SaveData {
            struct LinkData {
                btScalar position{};
                btScalar velocity{};
            };
            btTransform baseTransform;
            btVector3 baseVelocity;
            boost::container::static_vector<LinkData, 32> links;
        };

        int mSaveSelection = 0;
        PeriodicTask mSaveTask;
        boost::circular_buffer<SaveData> mSaveHistory;

        template<typename T, typename... Args>
        auto makeBulletObject(auto& vector, Args&&... args) -> T* {
            auto pointer = std::make_unique<T>(std::forward<Args>(args)...);
            auto* rawPointer = pointer.get();
            vector.emplace_back(std::move(pointer));
            return rawPointer;
        }

        // Scene

        std::unordered_map<std::string, URDF> mUrdfs;

        auto getUrdf(std::string const& name) -> std::optional<std::reference_wrapper<URDF>> {
            auto it = mUrdfs.find(name);
            if (it == mUrdfs.end()) return std::nullopt;

            return it->second;
        }

        SE3 mCameraInWorld{R3{-2.0, 0.0, 0.0}, SO3{}};

        std::vector<Camera> mCameras;

        static constexpr float NEAR = 0.1f;
        static constexpr float FAR = 1000.0f;

        // Other

        std::thread mRunThread;

        LoopProfiler mLoopProfiler{"Simulator"};

        auto cameraUpdate(Camera& camera, wgpu::CommandEncoder& encoder, wgpu::RenderPassDescriptor const& passDescriptor) -> void;

        auto gpsAndImusUpdate() -> void;

        auto linksToTfUpdate() -> void;

        auto keyCallback(int key, int scancode, int action, int mods) -> void;

        auto frameBufferResizedCallback(int width, int height) -> void;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override;

        auto initWindow() -> void;

        auto initRender() -> void;

        auto renderModel(wgpu::RenderPassEncoder& pass, Model& model, Uniform<ModelUniforms>& uniforms, SIM3 const& modelToWorld, bool isRoverCamera = false) -> void;

        auto initPhysics() -> void;

        auto parseParams() -> void;

        auto onInit() -> void override;

        auto run() -> void;

        auto centerCursor() -> void;

        auto freeLook(Clock::duration dt) -> void;

        auto userControls(Clock::duration dt) -> void;

        auto renderModels(wgpu::RenderPassEncoder& pass) -> void;

        auto renderWireframeColliders() -> void;

        auto renderUpdate() -> void;

        auto guiUpdate(wgpu::RenderPassEncoder& pass) -> void;

        auto physicsUpdate(Clock::duration dt) -> void;

        auto twistCallback(geometry_msgs::Twist::ConstPtr const& twist) -> void;

        auto jointPositionsCallback(Position::ConstPtr const& positions) -> void;

        auto makeTextureAndView(int width, int height, wgpu::TextureFormat const& format, wgpu::TextureUsage const& usage, wgpu::TextureAspect const& aspect) -> std::pair<wgpu::Texture, wgpu::TextureView>;

        auto makeFramebuffers(int width, int height) -> void;
    };

    auto uriToPath(std::string_view uri) -> std::filesystem::path;

    auto performXacro(std::filesystem::path const& path) -> std::string;

    auto readTextFile(std::filesystem::path const& path) -> std::string;

    auto readTexture(std::filesystem::path const& textureFileName) -> cv::Mat;

    auto urdfPoseToBtTransform(urdf::Pose const& pose) -> btTransform;

    auto btTransformToSim3(btTransform const& transform, btVector3 const& scale) -> SIM3;

    auto btTransformToSe3(btTransform const& transform) -> SE3;

    auto computeCameraToClip(float fovY, float aspect, float zNear, float zFar) -> Eigen::Matrix4f;

} // namespace mrover
