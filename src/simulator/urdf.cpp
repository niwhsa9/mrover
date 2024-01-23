#include "simulator.hpp"

namespace mrover {

    constexpr float MASS_MULTIPLIER = 1.0f;
    constexpr float DISTANCE_MULTIPLIER = 1.0f;
    constexpr float INERTIA_MULTIPLIER = MASS_MULTIPLIER * DISTANCE_MULTIPLIER * DISTANCE_MULTIPLIER;

    auto urdfPosToBtPos(urdf::Vector3 const& vec) -> btVector3 {
        return btVector3{static_cast<btScalar>(vec.x), static_cast<btScalar>(vec.y), static_cast<btScalar>(vec.z)} * DISTANCE_MULTIPLIER;
    }

    auto urdfQuatToBtQuat(urdf::Rotation const& quat) -> btQuaternion {
        return btQuaternion{static_cast<btScalar>(quat.x), static_cast<btScalar>(quat.y), static_cast<btScalar>(quat.z), static_cast<btScalar>(quat.w)};
    }

    auto urdfPoseToBtTransform(urdf::Pose const& pose) -> btTransform {
        return btTransform{urdfQuatToBtQuat(pose.rotation), urdfPosToBtPos(pose.position)};
    }

    auto urdfDistToBtDist(double scalar) -> btScalar {
        return static_cast<btScalar>(scalar) * DISTANCE_MULTIPLIER;
    }

    auto urdfMassToBtMass(double scalar) -> btScalar {
        return static_cast<btScalar>(scalar) * MASS_MULTIPLIER;
    }

    auto urdfInertiaToBtInertia(urdf::InertialSharedPtr const& inertia) -> btVector3 {
        return btVector3{static_cast<btScalar>(inertia->ixx), static_cast<btScalar>(inertia->iyy), static_cast<btScalar>(inertia->izz)} * INERTIA_MULTIPLIER;
    }

    auto URDF::makeCollisionShapeForLink(SimulatorNodelet& simulator, urdf::LinkConstSharedPtr const& link) -> btCollisionShape* {
        boost::container::static_vector<std::pair<btCollisionShape*, btTransform>, 4> shapes;
        for (urdf::CollisionSharedPtr const& collision: link->collision_array) {
            if (!collision->geometry) throw std::invalid_argument{"Collision has no geometry"};

            switch (collision->geometry->type) {
                case urdf::Geometry::BOX: {
                    auto box = std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
                    assert(box);

                    btVector3 boxHalfExtents = urdfPosToBtPos(box->dim) / 2;
                    shapes.emplace_back(simulator.makeBulletObject<btBoxShape>(simulator.mCollisionShapes, boxHalfExtents), urdfPoseToBtTransform(collision->origin));
                    break;
                }
                case urdf::Geometry::SPHERE: {
                    auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
                    assert(sphere);

                    btScalar radius = urdfDistToBtDist(sphere->radius);
                    shapes.emplace_back(simulator.makeBulletObject<btSphereShape>(simulator.mCollisionShapes, radius), urdfPoseToBtTransform(collision->origin));
                    break;
                }
                case urdf::Geometry::CYLINDER: {
                    auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
                    assert(cylinder);

                    btScalar radius = urdfDistToBtDist(cylinder->radius);
                    btScalar halfLength = urdfDistToBtDist(cylinder->length) / 2;
                    btVector3 cylinderHalfExtents{radius, radius, halfLength};
                    shapes.emplace_back(simulator.makeBulletObject<btCylinderShapeZ>(simulator.mCollisionShapes, cylinderHalfExtents), urdfPoseToBtTransform(collision->origin));
                    break;
                }
                case urdf::Geometry::MESH: {
                    auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);

                    assert(mesh);

                    std::string const& fileUri = mesh->filename;
                    auto [it, was_inserted] = simulator.mUriToModel.try_emplace(fileUri, fileUri);
                    Model& model = it->second;

                    model.waitMeshes();

                    if (model.meshes.size() != 1) throw std::invalid_argument{"Mesh collider must be constructed from exactly one mesh"};

                    Model::Mesh const& meshData = model.meshes.front();

                    auto* triangleMesh = new btTriangleMesh{};
                    triangleMesh->preallocateVertices(static_cast<int>(meshData.vertices.data.size()));
                    triangleMesh->preallocateIndices(static_cast<int>(meshData.indices.data.size()));
                    for (std::size_t i = 0; i < meshData.indices.data.size(); i += 3) {
                        Eigen::Vector3f v0 = meshData.vertices.data[meshData.indices.data[i + 0]];
                        Eigen::Vector3f v1 = meshData.vertices.data[meshData.indices.data[i + 1]];
                        Eigen::Vector3f v2 = meshData.vertices.data[meshData.indices.data[i + 2]];
                        triangleMesh->addTriangle(btVector3{v0.x(), v0.y(), v0.z()}, btVector3{v1.x(), v1.y(), v1.z()}, btVector3{v2.x(), v2.y(), v2.z()});
                    }
                    auto* meshShape = simulator.makeBulletObject<btBvhTriangleMeshShape>(simulator.mCollisionShapes, triangleMesh, true);
                    // TODO(quintin): Configure this in the URDF
                    meshShape->setMargin(0.01);
                    shapes.emplace_back(meshShape, urdfPoseToBtTransform(collision->origin));
                    simulator.mMeshToUri.emplace(meshShape, fileUri);
                    break;
                }
                default:
                    throw std::invalid_argument{"Unsupported collision type"};
            }
        }
        btCollisionShape* finalShape;
        switch (shapes.size()) {
            case 0:
                finalShape = simulator.makeBulletObject<btEmptyShape>(simulator.mCollisionShapes);
                break;
            default:
                auto* compoundShape = simulator.makeBulletObject<btCompoundShape>(simulator.mCollisionShapes);
                for (auto const& [shape, transform]: shapes) {
                    compoundShape->addChildShape(transform, shape);
                }
                finalShape = compoundShape;
                break;
        }
        return finalShape;
    }

    URDF::URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init) {
        auto urdfUri = xmlRpcValueToTypeOrDefault<std::string>(init, "uri");
        if (!model.initString(performXacro(uriToPath(urdfUri)))) throw std::runtime_error{std::format("Failed to parse URDF from URI: {}", urdfUri)};

        name = xmlRpcValueToTypeOrDefault<std::string>(init, "name");

        auto rootLinkInMap = btTransform::getIdentity();
        if (init.hasMember("translation")) {
            std::array<double, 3> translation = xmlRpcValueToNumberArray<3>(init, "translation");
            rootLinkInMap.setOrigin(btVector3{urdfDistToBtDist(translation[0]), urdfDistToBtDist(translation[1]), urdfDistToBtDist(translation[2])});
        }
        if (init.hasMember("rotation")) {
            std::array<double, 4> rotation = xmlRpcValueToNumberArray<4>(init, "rotation");
            rootLinkInMap.setRotation(btQuaternion{static_cast<btScalar>(rotation[0]), static_cast<btScalar>(rotation[1]), static_cast<btScalar>(rotation[2]), static_cast<btScalar>(rotation[3])});
        }

        std::size_t multiBodyLinkCount = model.links_.size() - 1; // Root link is treated separately by multibody, so subtract it off
        auto* multiBody = physics = simulator.makeBulletObject<btMultiBody>(simulator.mMultiBodies, multiBodyLinkCount, 0, btVector3{0, 0, 0}, false, false);
        multiBody->setBaseWorldTransform(rootLinkInMap);

        std::vector<btMultiBodyLinkCollider*> collidersToFinalize;
        std::vector<btMultiBodyConstraint*> constraintsToFinalize;

        auto traverse = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
            ROS_INFO_STREAM(std::format("Processing link: {}", link->name));

            auto linkIndex = static_cast<int>(linkNameToMeta.size()) - 1;
            auto [it, was_inserted] = linkNameToMeta.emplace(link->name, linkIndex);
            assert(was_inserted);

            if (link->name.contains("camera"sv)) {
                Camera camera = makeCameraForLink(simulator, &multiBody->getLink(linkIndex));
                if (link->name.contains("zed")) {
                    camera.pub = simulator.mNh.advertise<sensor_msgs::PointCloud2>("camera/left/points", 1);
                    simulator.mStereoCameras.emplace_back(std::move(camera));
                } else {
                    camera.pub = simulator.mNh.advertise<sensor_msgs::Image>("long_range_image", 1);
                    simulator.mCameras.push_back(std::move(camera));
                }
            }

            for (urdf::VisualSharedPtr const& visual: link->visual_array) {
                switch (visual->geometry->type) {
                    case urdf::Geometry::MESH: {
                        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
                        std::string const& fileUri = mesh->filename;
                        simulator.mUriToModel.try_emplace(fileUri, fileUri);
                        break;
                    }
                    default: {
                        ROS_WARN_STREAM("Currently only mesh visuals are supported");
                    }
                }
            }

            auto* collider = simulator.makeBulletObject<btMultiBodyLinkCollider>(simulator.mMultibodyCollider, multiBody, linkIndex);
            collider->setFriction(1);
            collidersToFinalize.push_back(collider);
            collider->setCollisionShape(makeCollisionShapeForLink(simulator, link));
            simulator.mDynamicsWorld->addCollisionObject(collider, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);

            btScalar mass = 1;
            btVector3 inertia{1, 1, 1}; // TODO(quintin): Is this a sane default?
            if (link->inertial) {
                mass = urdfMassToBtMass(link->inertial->mass);
                inertia = urdfInertiaToBtInertia(link->inertial);
            }

            if (urdf::JointConstSharedPtr parentJoint = link->parent_joint) {
                int parentIndex = linkNameToMeta.at(parentJoint->parent_link_name).index;
                btTransform jointInParent = urdfPoseToBtTransform(parentJoint->parent_to_joint_origin_transform);
                btTransform comInJoint = link->inertial ? urdfPoseToBtTransform(link->inertial->origin) : btTransform::getIdentity();
                btVector3 axisInJoint = urdfPosToBtPos(parentJoint->axis);

                switch (parentJoint->type) {
                    case urdf::Joint::FIXED: {
                        ROS_INFO_STREAM(std::format("Fixed joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        multiBody->setupFixed(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), jointInParent.getOrigin(), comInJoint.getOrigin());
                        break;
                    }
                    case urdf::Joint::CONTINUOUS:
                    case urdf::Joint::REVOLUTE: {
                        ROS_INFO_STREAM(std::format("Rotating joint {}: {} ({}) <-> {} ({})", parentJoint->name, parentJoint->parent_link_name, parentIndex, parentJoint->child_link_name, linkIndex));
                        multiBody->setupRevolute(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), axisInJoint, jointInParent.getOrigin(), comInJoint.getOrigin(), true);
                        if (parentJoint->type == urdf::Joint::REVOLUTE) {
                            auto lower = static_cast<btScalar>(parentJoint->limits->lower), upper = static_cast<btScalar>(parentJoint->limits->upper);
                            auto* limitConstraint = simulator.makeBulletObject<btMultiBodyJointLimitConstraint>(simulator.mMultibodyConstraints, multiBody, linkIndex, lower, upper);
                            constraintsToFinalize.push_back(limitConstraint);
                        }

                        if (link->name.contains("wheel"sv)) {
                            ROS_INFO_STREAM("\tGear");

                            // auto* gearConstraint = simulator.makeBulletObject<btMultiBodyGearConstraint>(simulator.mMultibodyConstraints, multiBody, parentIndex, multiBody, linkIndex, btVector3{}, btVector3{}, btMatrix3x3{}, btMatrix3x3{});
                            // gearConstraint->setMaxAppliedImpulse(10000);
                            // gearConstraint->setGearRatio(50);
                            // gearConstraint->setErp(0.2);
                            // constraintsToFinalize.push_back(gearConstraint);

                            collider->setRollingFriction(0.0);
                            collider->setSpinningFriction(0.0);
                            collider->setFriction(0.8);
                            collider->setContactStiffnessAndDamping(30000, 1000);
                        }
                        break;
                    }
                    case urdf::Joint::PRISMATIC: {
                        ROS_INFO_STREAM(std::format("Prismatic joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        multiBody->setupPrismatic(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), axisInJoint, jointInParent.getOrigin(), comInJoint.getOrigin(), true);
                        break;
                    }
                    default:
                        throw std::invalid_argument{"Unsupported joint type"};
                }
                switch (parentJoint->type) {
                    case urdf::Joint::CONTINUOUS:
                    case urdf::Joint::REVOLUTE:
                    case urdf::Joint::PRISMATIC: {
                        ROS_INFO_STREAM("\tMotor");
                        auto* motor = simulator.makeBulletObject<btMultiBodyJointMotor>(simulator.mMultibodyConstraints, multiBody, linkIndex, 0, 0);
                        constraintsToFinalize.push_back(motor);
                        multiBody->getLink(linkIndex).m_userPtr = motor;
                        break;
                    }
                    default:
                        break;
                }

                multiBody->getLink(linkIndex).m_collider = collider; // Bullet WHY? Why is this not exposed via a function call? This took a LONG time to figure out btw.
            } else {
                multiBody->setBaseMass(mass);
                multiBody->setBaseInertia(inertia);
                multiBody->setBaseCollider(collider);
            }

            it->second.collisionUniforms.resize(link->collision_array.size());
            it->second.visualUniforms.resize(link->visual_array.size());

            for (urdf::LinkConstSharedPtr childLink: link->child_links) {
                self(self, childLink);
            }
        };

        traverse(traverse, model.getRoot());

        multiBody->finalizeMultiDof();
        btAlignedObjectArray<btQuaternion> q;
        btAlignedObjectArray<btVector3> m;
        multiBody->forwardKinematics(q, m);
        multiBody->updateCollisionObjectWorldTransforms(q, m);
        simulator.mDynamicsWorld->addMultiBody(multiBody);

        for (btMultiBodyConstraint* constraint: constraintsToFinalize) {
            constraint->finalizeMultiDof();
            simulator.mDynamicsWorld->addMultiBodyConstraint(constraint);
        }
    }

} // namespace mrover
