/*=========================================================================

   Library: iMSTK

   Copyright (c) Kitware, Inc. & Center for Modeling, Simulation,
   & Imaging in Medicine, Rensselaer Polytechnic Institute.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0.txt

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=========================================================================*/

#include "imstkCamera.h"
#include "imstkDirectionalLight.h"
#include "imstkImageData.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
#include "imstkLineMesh.h"
#include "imstkMeshIO.h"
#include "imstkMeshToMeshBruteForceCD.h"
#include "imstkMouseControl.h"
#include "imstkMouseSceneControl.h"
#include "imstkNew.h"
#include "imstkOneToOneMap.h"

#include "imstkPbdBaryPointToPointConstraint.h"

#include "imstkPbdModel.h"
#include "imstkPbdObject.h"
#include "imstkPbdObjectCollision.h"
// #include "imstkPbdObjectPicking.h"
// #include "imstkPbdPickingCH.h"
#include "imstkPbdObjectGrasping.h"
#include "imstkRbdConstraint.h"
#include "imstkRenderMaterial.h"
#include "imstkRigidBodyModel2.h"
#include "imstkRigidObject2.h"
#include "imstkScene.h"
#include "imstkSceneManager.h"
#include "imstkSimulationManager.h"
#include "imstkSurfaceMesh.h"
#include "imstkTetrahedralMesh.h"
#include "imstkVisualModel.h"
#include "imstkVTKViewer.h"
#include "imstkCapsule.h"
#include "imstkSphere.h"
#include "imstkCollisionHandling.h"
#include "imstkPbdObjectCollision.h"
#include "imstkPointSetToCapsuleCD.h"
#include "imstkSurfaceMeshToCapsuleCD.h"

#include "imstkTaskNode.h"
#include "imstkTaskGraph.h"
#include "imstkCollisionDetectionAlgorithm.h"
#include "imstkMouseDeviceClient.h"

#include "PbdRigidBaryPointToPointConstraint.h"

// If two-way coupling is used haptic forces can be felt when the tool
// hits the tissue
//#define TWOWAY_COUPLING

// Whether to use FEM or volume+distance constraints
#define USE_FEM

#ifdef TWOWAY_COUPLING
#include "imstkPbdRigidObjectCollision.h"
#else
#include "imstkPbdObjectCollision.h"
#endif

#ifdef iMSTK_USE_OPENHAPTICS
#include "imstkHapticDeviceManager.h"
#include "imstkHapticDeviceClient.h"
#include "imstkRigidObjectController.h"
#endif

using namespace imstk;

class PbdRigidObjectGrasping : public PbdObjectGrasping
{
protected:
    std::shared_ptr<RigidObject2> m_rbdObj = nullptr;
    std::shared_ptr<PbdObject> m_pbdObj = nullptr;

public:
    PbdRigidObjectGrasping(
        std::shared_ptr<PbdObject> obj1,
        std::shared_ptr<RigidObject2> obj2) :
        PbdObjectGrasping(obj1), m_rbdObj(obj2)
    {
        m_pbdObj = obj1;

        m_taskGraph->addNode(m_pickingNode);
        m_taskGraph->addNode(obj2->getRigidBodyModel2()->getSolveNode());
    }
    ~PbdRigidObjectGrasping() override = default;

    const std::string getTypeName() const override { return "PbdRigidObjectGrasping"; }

    void updatePicking()
    {
        PbdObjectGrasping::updatePicking();

        // LOG(INFO) << "Before PBDRBD constraints";
        for (int i = 0; i < m_constraints.size(); i++)
        {
            auto constraint = std::dynamic_pointer_cast<PbdRigidBaryPointToPointConstraint>(m_constraints[i]);

            constraint->compute(m_rbdObj->getRigidBodyModel2()->getConfig()->m_dt);
            m_rbdObj->getRigidBodyModel2()->addConstraint(constraint);
        }
    }

    void addConstraint(
        const std::vector<VertexMassPair>& ptsA,
        const std::vector<double>& weightsA,
        const std::vector<VertexMassPair>& ptsB,
        const std::vector<double>& weightsB,
        double stiffnessA, double stiffnessB)
        override
    {
        // Create constraint
        auto constraint = std::make_shared<PbdRigidBaryPointToPointConstraint>(m_rbdObj->getRigidBody());
        constraint->initConstraint(
            ptsA,
            weightsA,
            ptsB,
            weightsB,
            stiffnessA,
            stiffnessB);
        // Add to constraints
        m_constraints.push_back(constraint);
    }

    void initGraphEdges(std::shared_ptr<TaskNode> source, std::shared_ptr<TaskNode> sink) override
    {
        LOG(INFO) << "Inside initGraphEdges";

        PbdObjectGrasping::initGraphEdges(source, sink);

        std::shared_ptr<PbdObject> pbdObj = m_pbdObj;

        std::shared_ptr<PbdModel> pbdModel = pbdObj->getPbdModel();
        std::shared_ptr<RigidBodyModel2> rbdModel = m_rbdObj->getRigidBodyModel2();


        m_taskGraph->addEdge(m_pickingNode, rbdModel->getSolveNode());
        m_taskGraph->addEdge(rbdModel->getSolveNode(), m_taskGraph->getSink());

        LOG(INFO) << "Leaving initGraphEdges";
    }
};

///
/// \brief Creates a tetraheral grid
/// \param physical dimension of tissue
/// \param dimensions of tetrahedral grid used for tissue
/// \param center of grid
///
static std::shared_ptr<TetrahedralMesh>
makeTetGrid(const Vec3d& size, const Vec3i& dim, const Vec3d& center)
{
    imstkNew<TetrahedralMesh> prismMesh;

    imstkNew<VecDataArray<double, 3>> verticesPtr(dim[0] * dim[1] * dim[2]);
    VecDataArray<double, 3>& vertices = *verticesPtr.get();
    const Vec3d                       dx = size.cwiseQuotient((dim - Vec3i(1, 1, 1)).cast<double>());
    for (int z = 0; z < dim[2]; z++)
    {
        for (int y = 0; y < dim[1]; y++)
        {
            for (int x = 0; x < dim[0]; x++)
            {
                vertices[x + dim[0] * (y + dim[1] * z)] = Vec3i(x, y, z).cast<double>().cwiseProduct(dx) - size * 0.5 + center;
            }
        }
    }

    // Add connectivity data
    imstkNew<VecDataArray<int, 4>> indicesPtr;
    VecDataArray<int, 4>& indices = *indicesPtr.get();
    for (int z = 0; z < dim[2] - 1; z++)
    {
        for (int y = 0; y < dim[1] - 1; y++)
        {
            for (int x = 0; x < dim[0] - 1; x++)
            {
                int cubeIndices[8] =
                {
                    x + dim[0] * (y + dim[1] * z),
                    (x + 1) + dim[0] * (y + dim[1] * z),
                    (x + 1) + dim[0] * (y + dim[1] * (z + 1)),
                    x + dim[0] * (y + dim[1] * (z + 1)),
                    x + dim[0] * ((y + 1) + dim[1] * z),
                    (x + 1) + dim[0] * ((y + 1) + dim[1] * z),
                    (x + 1) + dim[0] * ((y + 1) + dim[1] * (z + 1)),
                    x + dim[0] * ((y + 1) + dim[1] * (z + 1))
                };

                // Alternate the pattern so the edges line up on the sides of each voxel
                if ((z % 2 ^ x % 2) ^ y % 2)
                {
                    indices.push_back(Vec4i(cubeIndices[0], cubeIndices[7], cubeIndices[5], cubeIndices[4]));
                    indices.push_back(Vec4i(cubeIndices[3], cubeIndices[7], cubeIndices[2], cubeIndices[0]));
                    indices.push_back(Vec4i(cubeIndices[2], cubeIndices[7], cubeIndices[5], cubeIndices[0]));
                    indices.push_back(Vec4i(cubeIndices[1], cubeIndices[2], cubeIndices[0], cubeIndices[5]));
                    indices.push_back(Vec4i(cubeIndices[2], cubeIndices[6], cubeIndices[7], cubeIndices[5]));
                }
                else
                {
                    indices.push_back(Vec4i(cubeIndices[3], cubeIndices[7], cubeIndices[6], cubeIndices[4]));
                    indices.push_back(Vec4i(cubeIndices[1], cubeIndices[3], cubeIndices[6], cubeIndices[4]));
                    indices.push_back(Vec4i(cubeIndices[3], cubeIndices[6], cubeIndices[2], cubeIndices[1]));
                    indices.push_back(Vec4i(cubeIndices[1], cubeIndices[6], cubeIndices[5], cubeIndices[4]));
                    indices.push_back(Vec4i(cubeIndices[0], cubeIndices[3], cubeIndices[1], cubeIndices[4]));
                }
            }
        }
    }

    imstkNew<VecDataArray<float, 2>> uvCoordsPtr(dim[0] * dim[1] * dim[2]);
    VecDataArray<float, 2>& uvCoords = *uvCoordsPtr.get();
    for (int z = 0; z < dim[2]; z++)
    {
        for (int y = 0; y < dim[1]; y++)
        {
            for (int x = 0; x < dim[0]; x++)
            {
                uvCoords[x + dim[0] * (y + dim[1] * z)] = Vec2f(static_cast<float>(x) / dim[0], static_cast<float>(z) / dim[2]) * 3.0;
            }
        }
    }

    // Ensure correct windings
    for (int i = 0; i < indices.size(); i++)
    {
        if (tetVolume(vertices[indices[i][0]], vertices[indices[i][1]], vertices[indices[i][2]], vertices[indices[i][3]]) < 0.0)
        {
            std::swap(indices[i][0], indices[i][2]);
        }
    }

    prismMesh->initialize(verticesPtr, indicesPtr);
    prismMesh->setVertexTCoords("uvs", uvCoordsPtr);

    return prismMesh;
}


static std::shared_ptr<PbdObject>
makePbdObjSurface(const std::string& name,
    const Vec3d& size,
    const Vec3i& dim,
    const Vec3d& center,
    const int numIter)
{
    imstkNew<PbdObject> prismObj(name);

    // Setup the Geometry
    std::shared_ptr<TetrahedralMesh> prismMesh = makeTetGrid(size, dim, center);
    std::shared_ptr<SurfaceMesh>     surfMesh = prismMesh->extractSurfaceMesh();

    // Setup the Parameters
    imstkNew<PbdModelConfig> pbdParams;

    // Use volume+distance constraints, worse results. More performant (can use larger mesh)
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Dihedral, 1.0);
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 1.0);

    pbdParams->m_doPartitioning = true;
    pbdParams->m_uniformMassValue = 0.05;
    pbdParams->m_gravity = Vec3d(0.0, 0.0, 0.0);
    pbdParams->m_dt = 0.005;
    pbdParams->m_iterations = numIter;
    pbdParams->m_viscousDampingCoeff = 0.003;

    // Fix the borders
    for (int vert_id = 0; vert_id < surfMesh->getNumVertices(); vert_id++)
    {
        auto position = surfMesh->getVertexPosition(vert_id);

        if (position(1) == -2.0) {
            pbdParams->m_fixedNodeIds.push_back(vert_id);
        }

    }

    // Setup the Model
    imstkNew<PbdModel> pbdModel;
    pbdModel->setModelGeometry(surfMesh);
    pbdModel->configure(pbdParams);

    // Setup the material
    imstkNew<RenderMaterial> material;
    material->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);

    // Add a visual model to render the surface of the tet mesh
    imstkNew<VisualModel> visualModel;
    visualModel->setGeometry(surfMesh);
    visualModel->setRenderMaterial(material);
    prismObj->addVisualModel(visualModel);


    // Setup the Object
    prismObj->setPhysicsGeometry(surfMesh);
    prismObj->setCollidingGeometry(surfMesh);
    prismObj->setDynamicalModel(pbdModel);

    return prismObj;
}

static std::shared_ptr<RigidObject2>
makeCapsuleToolObj()
{
    // Set up rigid body model
    std::shared_ptr<RigidBodyModel2> rbdModel = std::make_shared<RigidBodyModel2>();
    rbdModel->getConfig()->m_gravity = Vec3d::Zero();
    rbdModel->getConfig()->m_maxNumIterations = 8;
    rbdModel->getConfig()->m_velocityDamping = 1.0;
    rbdModel->getConfig()->m_angularVelocityDamping = 1.0;
    rbdModel->getConfig()->m_maxNumConstraints = 40;


    auto toolGeometry = std::make_shared<Capsule>();
    toolGeometry->setRadius(0.5);
    toolGeometry->setLength(1);
    toolGeometry->setPosition(Vec3d(0.0, 0.0, 0.0));
    toolGeometry->setOrientation(Quatd(0.707, 0.0, 0.0, 0.707));

    std::shared_ptr<RigidObject2> toolObj = std::make_shared<RigidObject2>("Tool");

    // Create the object
    toolObj->setVisualGeometry(toolGeometry);
    toolObj->setPhysicsGeometry(toolGeometry);
    toolObj->setCollidingGeometry(toolGeometry);
    toolObj->setDynamicalModel(rbdModel);
    toolObj->getRigidBody()->m_mass = 1.0;
    toolObj->getRigidBody()->m_intertiaTensor = Mat3d::Identity() * 1.0;
    toolObj->getRigidBody()->m_initPos = Vec3d(0.0, 1.0, 2.0);

    toolObj->getVisualModel(0)->getRenderMaterial()->setOpacity(0.5);

    return toolObj;
}


///
/// \brief This example demonstrates collision interaction with a 3d pbd
/// simulated tissue (tetrahedral)
///
int
main()
{
    // Setup logger (write to file and stdout)
    Logger::startLogger();

    // Setup the scene
    imstkNew<Scene> scene("PBDGrasping");
    scene->getActiveCamera()->setPosition(0.12, 4.51, 16.51);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.0, 0.0);
    scene->getActiveCamera()->setViewUp(0.0, 0.96, -0.28);

    scene->getConfig()->writeTaskGraph = true;

    // Setup a tissue
    std::shared_ptr<PbdObject> PbdObj = makePbdObjSurface("Tissue",
        Vec3d(4.0, 4.0, 4.0), Vec3i(5, 5, 5), Vec3d(0.0, 0.0, 0.0), 8);
    scene->addSceneObject(PbdObj);

    std::shared_ptr<RigidObject2> toolObj = makeCapsuleToolObj();
    scene->addSceneObject(toolObj);

    auto rbdGhost = std::make_shared<SceneObject>("ghost");
    auto ghostCapsule = std::make_shared<Capsule>();
    ghostCapsule->setRadius(0.5);
    ghostCapsule->setLength(1);
    ghostCapsule->setPosition(Vec3d(0.0, 0.0, 0.0));
    ghostCapsule->setOrientation(Quatd(0.707, 0.0, 0.0, 0.707));
    rbdGhost->setVisualGeometry(ghostCapsule);


    std::shared_ptr<RenderMaterial> ghostMat =
        std::make_shared<RenderMaterial>(*toolObj->getVisualModel(0)->getRenderMaterial());
    ghostMat->setOpacity(0.4);
    ghostMat->setColor(Color::Red);
    rbdGhost->getVisualModel(0)->setRenderMaterial(ghostMat);
    scene->addSceneObject(rbdGhost);

    // Add collision
    // auto pbdToolCollision = std::make_shared<PbdObjectCollision>(PbdObj, toolObj, "SurfaceMeshToCapsuleCD");
    // scene->addInteraction(pbdToolCollision);

    // Create new picking with constraints
    auto toolPicking = std::make_shared<PbdRigidObjectGrasping>(PbdObj, toolObj);
    scene->addInteraction(toolPicking);


    // auto toolPicking = std::make_shared<PbdObjectGrasping>(PbdObj);
    // scene->addInteraction(toolPicking);

    // Light
    imstkNew<DirectionalLight> light;
    light->setFocalPoint(Vec3d(5.0, -8.0, -5.0));
    light->setIntensity(1);
    scene->addLight("Light", light);

    // scene->getConfig()->writeTaskGraph = true;

    // Run the simulation
    {
        // Setup a viewer to render
        imstkNew<VTKViewer> viewer;
        viewer->setActiveScene(scene);
        viewer->setVtkLoggerMode(VTKViewer::VTKLoggerMode::MUTE);

        // Setup a scene manager to advance the scene
        imstkNew<SceneManager> sceneManager;
        sceneManager->setActiveScene(scene);
        sceneManager->pause(); // Start simulation paused

        imstkNew<SimulationManager> driver;
        driver->addModule(viewer);
        driver->addModule(sceneManager);
        driver->setDesiredDt(0.002);

#ifdef iMSTK_USE_OPENHAPTICS
        imstkNew<HapticDeviceManager> hapticManager;
        hapticManager->setSleepDelay(1.0); // Delay for 1ms (haptics thread is limited to max 1000hz)
        std::shared_ptr<HapticDeviceClient> hapticDeviceClient = hapticManager->makeDeviceClient();
        driver->addModule(hapticManager);

        imstkNew<RigidObjectController> controller(toolObj, hapticDeviceClient);
        controller->setTranslationScaling(0.05);
        controller->setLinearKs(10000.0);
        controller->setAngularKs(1000.0);
        //controller->setAngularKs(0.0);
        controller->setUseCritDamping(true);
        controller->setForceScaling(0.001);
        controller->setSmoothingKernelSize(10);
        controller->setUseForceSmoothening(true);
        scene->addController(controller);

        connect<Event>(sceneManager, &SceneManager::postUpdate,
            [&](Event*)
            {
                ghostMat->setOpacity(std::min(1.0, controller->getDeviceForce().norm() / 15.0));

                // Also apply controller transform to ghost geometry
                ghostCapsule->setTranslation(controller->getPosition());
                ghostCapsule->setRotation(controller->getOrientation());
                ghostCapsule->updatePostTransformData();
                ghostCapsule->postModified();
            });
        connect<ButtonEvent>(hapticDeviceClient, &HapticDeviceClient::buttonStateChanged,
            [&](ButtonEvent* e)
            {
                if (e->m_buttonState == BUTTON_PRESSED)
                {
                    if (e->m_button == 1)
                    {
                        toolPicking->beginVertexGrasp(std::dynamic_pointer_cast<Capsule>(toolObj->getCollidingGeometry()));
                    }
                }
                else if (e->m_buttonState == BUTTON_RELEASED)
                {
                    if (e->m_button == 1)
                    {
                        toolPicking->endGrasp();
                    }
                }
            });
#else
        connect<Event>(sceneManager, &SceneManager::postUpdate,
            [&](Event*)
            {
                const Vec2d mousePos = viewer->getMouseDevice()->getPos();
                const Vec3d worldPos = Vec3d(mousePos[0] - 0.5, mousePos[1] - 0.5, 0.0) * 10.0;

                const Vec3d fS = (worldPos - toolObj->getRigidBody()->getPosition()) * 100000.0; // Spring force
                const Vec3d fD = -toolObj->getRigidBody()->getVelocity() * 100.0;              // Spring damping

                (*toolObj->getRigidBody()->m_force) += (fS + fD);

                // Also apply controller transform to ghost geometry
                ghostCapsule->setTranslation(worldPos);
                ghostCapsule->setRotation(Mat3d::Identity());
                ghostCapsule->updatePostTransformData();
                ghostCapsule->postModified();
            });

        // Add click event and side effects
        connect<Event>(viewer->getMouseDevice(), &MouseDeviceClient::mouseButtonPress,
            [&](Event*)
            {
                toolPicking->beginVertexGrasp(std::dynamic_pointer_cast<AnalyticalGeometry>(toolObj->getCollidingGeometry()));
            });

        // Add click event and side effects
        connect<Event>(viewer->getMouseDevice(), &MouseDeviceClient::mouseButtonRelease,
            [&](Event*)
            {
                // When click, grab all verticese associated with the triangle 
                // faces in contact and force motion with tool. 
                toolPicking->endGrasp();
            });
#endif

        // Add mouse and keyboard controls to the viewer
        imstkNew<MouseSceneControl> mouseControl(viewer->getMouseDevice());
        mouseControl->setSceneManager(sceneManager);
        viewer->addControl(mouseControl);

        imstkNew<KeyboardSceneControl> keyControl(viewer->getKeyboardDevice());
        keyControl->setSceneManager(sceneManager);
        keyControl->setModuleDriver(driver);
        viewer->addControl(keyControl);

         connect<Event>(sceneManager, &SceneManager::postUpdate, [&](Event*)
             {
                 // Simulate the cube in real time
                 PbdObj->getPbdModel()->getConfig()->m_dt = sceneManager->getDt();
                 toolObj->getRigidBodyModel2()->getConfig()->m_dt = sceneManager->getDt();
             });

        driver->start();
    }

    return 0;
}