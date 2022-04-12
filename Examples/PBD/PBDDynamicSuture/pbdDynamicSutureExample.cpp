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
#include "imstkCapsule.h"
#include "imstkCollisionDetectionAlgorithm.h"
#include "imstkCollisionHandling.h"
#include "imstkDebugGeometryObject.h"
#include "imstkDirectionalLight.h"
#include "imstkIsometricMap.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
#include "imstkLineMesh.h"
#include "imstkMeshIO.h"
#include "imstkMouseDeviceClient.h"
#include "imstkMouseSceneControl.h"
#include "imstkPbdModel.h"
#include "imstkPbdObject.h"
#include "imstkPbdObjectCollision.h"
#include "imstkPbdObjectGrasping.h"
#include "imstkRbdConstraint.h"
#include "imstkPointwiseMap.h"
#include "imstkRenderMaterial.h"
#include "imstkRigidBodyModel2.h"
#include "imstkRigidObject2.h"
#include "imstkRigidObjectController.h"
#include "imstkScene.h"
#include "imstkSceneManager.h"
#include "imstkSimulationManager.h"
#include "imstkSurfaceMesh.h"
#include "imstkTetrahedralMesh.h"
#include "imstkVisualModel.h"
#include "imstkVRPNDeviceClient.h"
#include "imstkVRPNDeviceManager.h"
#include "imstkVTKViewer.h"

#include "NeedleInteraction.h"
#include "NeedleObject.h"

#ifdef iMSTK_USE_OPENHAPTICS
#include "imstkHapticDeviceManager.h"
#include "imstkHapticDeviceClient.h"
#include "imstkRigidObjectController.h"
#endif



using namespace imstk;

std::shared_ptr<PbdObject> createTissueHole(std::shared_ptr<TetrahedralMesh> tetMesh);

struct Input
{
    std::string meshFileName;
};

Input input;

// Create tissue object to stitch
std::shared_ptr<PbdObject> 
createTissueHole(std::shared_ptr<TetrahedralMesh> tetMesh)
{

    std::shared_ptr<SurfaceMesh> surfMesh = tetMesh->extractSurfaceMesh();

    auto pbdObject = std::make_shared<PbdObject>("meshHole");

    auto pbdParams = std::make_shared<PbdModelConfig>();
    
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 10.0);
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Volume, 10.0);
    pbdParams->m_doPartitioning   = false;
    pbdParams->m_uniformMassValue = 0.1;
    pbdParams->m_gravity    = Vec3d(0.0, 0.0, -0.1);
    pbdParams->m_dt = 0.01;
    pbdParams->m_iterations = 5;
    pbdParams->m_viscousDampingCoeff = 0.01;


    // Fix the borders
    for (int vert_id = 0; vert_id < surfMesh->getNumVertices(); vert_id++)
    {
        auto position = tetMesh->getVertexPosition(vert_id);
        if (std::fabs(1.40984-std::fabs(position(1))) <= 1E-4)
        {
            pbdParams->m_fixedNodeIds.push_back(vert_id);
        }
    }

    // Setup the Model
    auto pbdModel = std::make_shared<PbdModel>();
    pbdModel->setModelGeometry(tetMesh);
    pbdModel->configure(pbdParams);

    // Setup the material
    auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);

    // Add a visual model to render the surface of the tet mesh
    auto visualModel = std::make_shared<VisualModel>();
    visualModel->setGeometry(tetMesh);
    visualModel->setRenderMaterial(material);
    pbdObject->addVisualModel(visualModel);

    // Setup the Object
    pbdObject->setPhysicsGeometry(tetMesh);
    pbdObject->setCollidingGeometry(surfMesh);
    pbdObject->setPhysicsToCollidingMap(std::make_shared<PointwiseMap>(tetMesh, surfMesh));
    pbdObject->setDynamicalModel(pbdModel);

    return pbdObject;
}

// // Capsule to test controllers
// static std::shared_ptr<RigidObject2>
// makeCapsuleToolObj()
// {
//     // Set up rigid body model
//     std::shared_ptr<RigidBodyModel2> rbdModel = std::make_shared<RigidBodyModel2>();
//     rbdModel->getConfig()->m_gravity = Vec3d::Zero();
//     rbdModel->getConfig()->m_maxNumIterations       = 8;
//     rbdModel->getConfig()->m_velocityDamping        = 1.0;
//     rbdModel->getConfig()->m_angularVelocityDamping = 1.0;
//     rbdModel->getConfig()->m_maxNumConstraints      = 40;

//     auto toolGeometry = std::make_shared<Capsule>();
//     toolGeometry->setRadius(0.5);
//     toolGeometry->setLength(1);
//     toolGeometry->setPosition(Vec3d(0.0, 0.0, 0.0));
//     toolGeometry->setOrientation(Quatd(0.707, 0.0, 0.0, 0.707));

//     std::shared_ptr<RigidObject2> toolObj = std::make_shared<RigidObject2>("Tool");

//     // Create the object
//     toolObj->setVisualGeometry(toolGeometry);
//     toolObj->setPhysicsGeometry(toolGeometry);
//     toolObj->setCollidingGeometry(toolGeometry);
//     toolObj->setDynamicalModel(rbdModel);
//     toolObj->getRigidBody()->m_mass = 1.0;
//     toolObj->getRigidBody()->m_intertiaTensor = Mat3d::Identity() * 1.0;
//     toolObj->getRigidBody()->m_initPos = Vec3d(0.0, 1.0, 2.0);

//     toolObj->getVisualModel(0)->getRenderMaterial()->setOpacity(0.5);

//     return toolObj;
// }

// // Needle for suturing
// static std::shared_ptr<SceneObject>
// makeToolObj(std::string name)
// {
//     auto surfMesh =
//         MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "/Surgical Instruments/Clamps/Gregory Suture Clamp/gregory_suture_clamp.obj");

//     auto toolObj = std::make_shared<SceneObject>(name);
//     toolObj->setVisualGeometry(surfMesh);
    
//     auto renderMaterial = std::make_shared<RenderMaterial>();
//     renderMaterial->setColor(Color::LightGray);
//     renderMaterial->setShadingModel(RenderMaterial::ShadingModel::PBR);
//     renderMaterial->setRoughness(0.5);
//     renderMaterial->setMetalness(1.0);
//     toolObj->getVisualModel(0)->setRenderMaterial(renderMaterial);

//     return toolObj;
// }

// Syringe geometry
static std::shared_ptr<NeedleObject>
makeToolObj()
{

    auto toolGeometry = std::make_shared<LineMesh>();
    auto verticesPtr = std::make_shared<VecDataArray<double, 3>>(2);
    (*verticesPtr)[0] = Vec3d(0.0, -0.05, 0.0);
    (*verticesPtr)[1] = Vec3d(0.0, 0.05, 0.0);

    auto indicesPtr = std::make_shared<VecDataArray<int, 2>>(1);
    (*indicesPtr)[0] = Vec2i(0, 1);
    toolGeometry->initialize(verticesPtr, indicesPtr);

    auto syringeMesh = MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "/Surgical Instruments/Syringes/Disposable_Syringe.stl");
    syringeMesh->rotate(Vec3d(1.0, 0.0, 0.0), -PI_2, Geometry::TransformType::ApplyToData);
    syringeMesh->translate(Vec3d(0.0, 4.4, 0.0), Geometry::TransformType::ApplyToData);
    syringeMesh->scale(0.0055, Geometry::TransformType::ApplyToData);
    syringeMesh->translate(Vec3d(0.0, 0.1, 0.0));

    auto toolObj = std::make_shared<NeedleObject>("NeedleRbdTool");
    toolObj->setVisualGeometry(syringeMesh);
    toolObj->setCollidingGeometry(toolGeometry);
    toolObj->setPhysicsGeometry(toolGeometry);
    toolObj->setPhysicsToVisualMap(std::make_shared<IsometricMap>(toolGeometry, syringeMesh));
    toolObj->getVisualModel(0)->getRenderMaterial()->setColor(Color(0.9, 0.9, 0.9));
    toolObj->getVisualModel(0)->getRenderMaterial()->setShadingModel(RenderMaterial::ShadingModel::PBR);
    toolObj->getVisualModel(0)->getRenderMaterial()->setRoughness(0.5);
    toolObj->getVisualModel(0)->getRenderMaterial()->setMetalness(1.0);
    toolObj->getVisualModel(0)->getRenderMaterial()->setIsDynamicMesh(false);

    std::shared_ptr<RigidBodyModel2> rbdModel = std::make_shared<RigidBodyModel2>();
    rbdModel->getConfig()->m_gravity = Vec3d::Zero();
    rbdModel->getConfig()->m_maxNumIterations = 5;
    toolObj->setDynamicalModel(rbdModel);

    toolObj->getRigidBody()->m_mass = 1.0;
    toolObj->getRigidBody()->m_intertiaTensor = Mat3d::Identity() * 10000.0;
    toolObj->getRigidBody()->m_initPos = Vec3d(0.0, 0.1, 0.0);

    return toolObj;
}

// ///
// /// \brief Create pbd string geometry
// ///
// static std::shared_ptr<LineMesh>
// makeStringGeometry(const Vec3d& pos, const Vec3d& dx, const int numVerts)
// {
//     // Create the geometry
//     auto stringGeometry = std::make_shared<LineMesh>();

//     auto verticesPtr = std::make_shared<VecDataArray<double, 3>>(numVerts);
//     VecDataArray<double, 3>&          vertices = *verticesPtr.get();
//     for (int i = 0; i < numVerts; i++)
//     {
//         vertices[i] = pos + dx * i;
//     }

//     // Add connectivity data
//     auto segmentsPtr = std::make_shared<VecDataArray<int, 2>>();
//     VecDataArray<int, 2>&          segments = *segmentsPtr.get();
//     for (int i = 0; i < numVerts - 1; i++)
//     {
//         segments.push_back(Vec2i(i, i + 1));
//     }

//     stringGeometry->initialize(verticesPtr, segmentsPtr);
//     return stringGeometry;
// }

// ///
// /// \brief Create pbd string object
// ///
// static std::shared_ptr<PbdObject>
// makePbdString(
//     const std::string& name,
//     const Vec3d& pos, const Vec3d& dir, const int numVerts,
//     const double stringLength)
// {
//     auto stringObj = std::make_shared<PbdObject>(name);

//     // Setup the Geometry
//     const double              dx = stringLength / numVerts;
//     std::shared_ptr<LineMesh> stringMesh = makeStringGeometry(pos, dir * dx, numVerts);

//     // Setup the Parameters
//     auto pbdParams = std::make_shared<PbdModelConfig>();
//     pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 100.0);
//     pbdParams->enableBendConstraint(100000.0, 1);
//     pbdParams->enableBendConstraint(100000.0, 2);
//     pbdParams->m_fixedNodeIds     = { 0, 1 };
//     pbdParams->m_uniformMassValue = 0.002 / numVerts; // grams
//     pbdParams->m_gravity = Vec3d(0.0, -9.8, 0.0);
//     pbdParams->m_dt      = 0.0005;                    // Overwritten for real time

//     // Requires large amounts of iterations the longer, a different
//     // solver would help
//     pbdParams->m_iterations = 100;
//     pbdParams->m_viscousDampingCoeff = 0.01;

//     // Setup the Model
//     auto pbdModel = std::make_shared<PbdModel>();
//     pbdModel->setModelGeometry(stringMesh);
//     pbdModel->configure(pbdParams);

//     // Setup the VisualModel
//     auto material = std::make_shared<RenderMaterial>();
//     material->setBackFaceCulling(false);
//     material->setColor(Color::Red);
//     material->setLineWidth(2.0);
//     material->setPointSize(6.0);
//     material->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);

//     auto visualModel = std::make_shared<VisualModel>();
//     visualModel->setGeometry(stringMesh);
//     visualModel->setRenderMaterial(material);

//     // Setup the Object
//     stringObj->addVisualModel(visualModel);
//     stringObj->setPhysicsGeometry(stringMesh);
//     stringObj->setCollidingGeometry(stringMesh);
//     stringObj->setDynamicalModel(pbdModel);

//     return stringObj;
// }


///
/// \brief This example demonstrates suturing of a hole in a tissue
///
int
main()
{
    // Setup logger (write to file and stdout)
    Logger::startLogger();

    input.meshFileName = iMSTK_DATA_ROOT "Tissues/tissue_hole.vtk";

    // Construct the scene
    auto scene = std::make_shared<Scene>("DynamicSuture");
    
    scene->getActiveCamera()->setPosition(0.0, 0.0, 10.0);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.0, 1.0);
    scene->getActiveCamera()->setViewUp(0.0, 1.0, 0.0);

    // Load a tetrahedral mesh
    std::shared_ptr<TetrahedralMesh> tetMesh = MeshIO::read<TetrahedralMesh>(input.meshFileName);
    CHECK(tetMesh != nullptr) << "Could not read mesh from file.";

    // Mesh with hole for suturing
    std::shared_ptr<PbdObject> tissueHole = createTissueHole(tetMesh);
    scene->addSceneObject(tissueHole);

    // Test Capsule
    // std::shared_ptr<RigidObject2> toolObj = makeCapsuleToolObj();
    // scene->addSceneObject(toolObj);

// Curved Needle
    // // Create the arc needle
    // auto needleObj = std::make_shared<NeedleObject>();
    // needleObj->setForceThreshold(2.0);
    // scene->addSceneObject(needleObj);

    // // Create the suture pbd-based string
    // const double               stringLength      = 0.2;
    // const int                  stringVertexCount = 30;
    // std::shared_ptr<PbdObject> sutureThreadObj   =
    //     makePbdString("SutureThread", Vec3d(0.0, 0.0, 0.018), Vec3d(0.0, 0.0, 1.0),
    //         stringVertexCount, stringLength);
    // scene->addSceneObject(sutureThreadObj);

    // // Create clamps that follow the needle around
    // std::shared_ptr<SceneObject> clampsObj = makeToolObj("Clamps");
    // scene->addSceneObject(clampsObj);

    // // Create ghost clamps to show real position of hand under virtual coupling
    // std::shared_ptr<SceneObject> ghostClampsObj = makeToolObj("GhostClamps");
    // ghostClampsObj->getVisualModel(0)->getRenderMaterial()->setColor(Color::Orange);
    // scene->addSceneObject(ghostClampsObj);

    // // Add point based collision between the tissue & suture thread
    // auto interaction = std::make_shared<PbdObjectCollision>(sutureThreadObj, tissueHole, "ImplicitGeometryToPointSetCD");
    // interaction->setFriction(0.0);
    // scene->addInteraction(interaction);

    // // Add needle constraining behaviour between the tissue & arc needle
    // auto needleInteraction = std::make_shared<NeedleInteraction>(tissueHole, needleObj);
    // scene->addInteraction(needleInteraction);

// Injector


    // Setup a tool for the user to move
    std::shared_ptr<NeedleObject> toolObj = makeToolObj();
    toolObj->setForceThreshold(15.0);
    scene->addSceneObject(toolObj);

    // Setup a debug ghost tool for virtual coupling
    auto ghostToolObj = std::make_shared<SceneObject>("ghostTool");
    {
        auto toolMesh = std::dynamic_pointer_cast<SurfaceMesh>(toolObj->getVisualGeometry());

        auto toolGhostMesh = std::make_shared<SurfaceMesh>();
        toolGhostMesh->initialize(
            std::make_shared<VecDataArray<double, 3>>(*toolMesh->getVertexPositions(Geometry::DataType::PreTransform)),
            std::make_shared<VecDataArray<int, 3>>(*toolMesh->getTriangleIndices()));
        ghostToolObj->setVisualGeometry(toolGhostMesh);
        ghostToolObj->getVisualModel(0)->getRenderMaterial()->setColor(Color::Orange);
        ghostToolObj->getVisualModel(0)->getRenderMaterial()->setLineWidth(5.0);
        ghostToolObj->getVisualModel(0)->getRenderMaterial()->setOpacity(0.3);
        ghostToolObj->getVisualModel(0)->getRenderMaterial()->setIsDynamicMesh(false);
    }
    scene->addSceneObject(ghostToolObj);

    // Setup a debug polygon soup for debug contact points
    auto debugGeomObj = std::make_shared<DebugGeometryObject>();
    debugGeomObj->setLineWidth(0.1);
    scene->addSceneObject(debugGeomObj);

    // This adds both contact and puncture functionality
    auto interaction = std::make_shared<NeedleInteraction>(tissueHole, toolObj);
    scene->addInteraction(interaction);



    /// \ IP address of the server.
    // const std::string serverIP = "localhost";

    /// \ the default VRPN port 38833
    // const int serverPort = 3883;

    // VRPN Server for 3D mouse control
    // auto server = std::make_shared<VRPNDeviceManager>(serverIP, serverPort);
    // auto client = server->makeDeviceClient("tracker0", VRPNTracker);

    // Add 3D mouse Controller for capsule
    // auto controller = std::make_shared<SceneObjectController>(toolObj, client);
    // controller->setTranslationScaling(1.0);
    // scene->addController(controller);

    // auto hapticManager = std::make_shared<HapticDeviceManager>();
    // hapticManager->setSleepDelay(1.0); // Delay for 1ms (haptics thread is limited to max 1000hz)
    // std::shared_ptr<HapticDeviceClient> hapticDeviceClient = hapticManager->makeDeviceClient();
   

    // auto hapController = std::make_shared<RigidObjectController>(toolObj, hapticDeviceClient);
    // hapController->setTranslationScaling(0.05);
    // hapController->setLinearKs(10000.0);
    // hapController->setAngularKs(1000.0);
    // //hapController->setAngularKs(0.0);
    // hapController->setUseCritDamping(true);
    // hapController->setForceScaling(0.001);
    // hapController->setSmoothingKernelSize(10);
    // hapController->setUseForceSmoothening(true);
    // scene->addController(hapController);

    // Light
    auto light = std::make_shared<DirectionalLight>();
    light->setFocalPoint(Vec3d(0.0, -1.0, -1.0));
    light->setIntensity(1.0);
    scene->addLight("light", light);

    scene->getConfig()->writeTaskGraph = true;
    

    // Run the simulation
    {
        // Setup a viewer to render
        auto viewer = std::make_shared<VTKViewer>();
        viewer->setActiveScene(scene);
        viewer->setDebugAxesLength(0.01, 0.01, 0.01);

        // Setup a scene manager to advance the scene
        auto sceneManager = std::make_shared<SceneManager>();
        sceneManager->setActiveScene(scene);
        sceneManager->pause(); // Start simulation paused

        // Setup a simulation manager to manage renders & scene updates
        auto driver = std::make_shared<SimulationManager>();
        driver->addModule(viewer);
        driver->addModule(sceneManager);
        driver->setDesiredDt(0.001); // 1ms, 1000hz

#ifdef iMSTK_USE_OPENHAPTICS

        auto hapticManager = std::make_shared<HapticDeviceManager>();
        hapticManager->setSleepDelay(1.0); // Delay for 1ms (haptics thread is limited to max 1000hz)
        std::shared_ptr<HapticDeviceClient> deviceClient = hapticManager->makeDeviceClient();

        auto hapController = std::make_shared<RigidObjectController>(toolObj, deviceClient);
        hapController->setTranslationScaling(0.05);
        hapController->setLinearKs(10000.0);
        hapController->setAngularKs(1000.0);
        //hapController->setAngularKs(0.0);
        hapController->setUseCritDamping(true);
        hapController->setForceScaling(0.001);
        hapController->setSmoothingKernelSize(10);
        hapController->setUseForceSmoothening(true);
        scene->addController(hapController);

        const double translationScaling = 0.001;
        const Vec3d  offset = Vec3d(0.05, -0.05, 0.0);
#else
        auto deviceClient = std::make_shared<MouseDeviceClient3D>(viewer->getMouseDevice());
        deviceClient->setOrientation(Quatd(Rotd(1.57, Vec3d(0.0, 1.0, 0.0))));
        const double translationScaling = 0.1;
        const Vec3d  offset = Vec3d(-0.05, -0.1, -0.005);

        connect<MouseEvent>(viewer->getMouseDevice(), &MouseDeviceClient::mouseScroll,
            [&](MouseEvent* e)
            {
                const Quatd delta = Quatd(Rotd(e->m_scrollDx * 0.1, Vec3d(0.0, 0.0, 1.0)));
                deviceClient->setOrientation(deviceClient->getOrientation() * delta);
            });
#endif
        auto controller = std::make_shared<RigidObjectController>(toolObj, deviceClient);
        controller->setTranslationOffset(offset);
        controller->setTranslationScaling(translationScaling);
        controller->setLinearKs(1000.0);
        controller->setAngularKs(10000000.0);
        controller->setUseCritDamping(true);
        controller->setForceScaling(0.2);
        controller->setSmoothingKernelSize(5);
        controller->setUseForceSmoothening(true);
        scene->addController(controller);

        // Update the timesteps for real time
        connect<Event>(sceneManager, &SceneManager::preUpdate,
            [&](Event*)
            {
                toolObj->getRigidBodyModel2()->getConfig()->m_dt = sceneManager->getDt();
                // sutureThreadObj->getPbdModel()->getConfig()->m_dt  = sceneManager->getDt();
            });
        // Constrain the first two vertices of the string to the needle
        // connect<Event>(sceneManager, &SceneManager::postUpdate,
        //     [&](Event*)
        //     {
        //         auto needleLineMesh = std::dynamic_pointer_cast<LineMesh>(toolObj->getPhysicsGeometry());
        //         auto sutureLineMesh = std::dynamic_pointer_cast<LineMesh>(sutureThreadObj->getPhysicsGeometry());
        //         (*sutureLineMesh->getVertexPositions())[1] = (*needleLineMesh->getVertexPositions())[0];
        //         (*sutureLineMesh->getVertexPositions())[0] = (*needleLineMesh->getVertexPositions())[1];
        //     });
        // // Transform the clamps relative to the needle
        // const Vec3d clampOffset = Vec3d(-0.009, 0.01, 0.001);
        // connect<Event>(sceneManager, &SceneManager::postUpdate,
        //     [&](Event*)
        //     {
        //         clampsObj->getVisualGeometry()->setTransform(
        //             toolObj->getVisualGeometry()->getTransform() *
        //             mat4dTranslate(clampOffset) *
        //             mat4dRotation(Rotd(PI, Vec3d(0.0, 1.0, 0.0))));
        //         clampsObj->getVisualGeometry()->postModified();
        //     });
        // // Transform the ghost tool clamps to show the real tool location
        // connect<Event>(sceneManager, &SceneManager::postUpdate,
        //     [&](Event*)
        //     {
        //         ghostClampsObj->getVisualGeometry()->setTransform(
        //             mat4dTranslate(controller->getPosition()) * mat4dRotation(controller->getOrientation()) *
        //             mat4dTranslate(clampOffset) *
        //             mat4dRotation(Rotd(PI, Vec3d(0.0, 1.0, 0.0))));
        //         ghostClampsObj->getVisualGeometry()->updatePostTransformData();
        //         ghostClampsObj->getVisualGeometry()->postModified();
        //         ghostClampsObj->getVisualModel(0)->getRenderMaterial()->setOpacity(std::min(1.0, controller->getDeviceForce().norm() / 5.0));
        //     });

        // Add mouse and keyboard controls to the viewer
        {
        
        // Add mouse and keyboard controls to the viewer
        auto mouseControl = std::make_shared<MouseSceneControl>(viewer->getMouseDevice());
        mouseControl->setSceneManager(sceneManager);
        viewer->addControl(mouseControl);

        auto keyControl = std::make_shared<KeyboardSceneControl>(viewer->getKeyboardDevice());
        keyControl->setSceneManager(sceneManager);
        keyControl->setModuleDriver(driver);
        viewer->addControl(keyControl);
        }

        driver->start();
    }


    //  // Run the simulation
    // {
    //     // Setup a viewer to render
    //     auto viewer = std::make_shared<VTKViewer>();
    //     viewer->setActiveScene(scene);
    //     viewer->setDebugAxesLength(0.01, 0.01, 0.01);

    //     // Setup a scene manager to advance the scene
    //     auto sceneManager = std::make_shared<SceneManager>();
    //     sceneManager->setActiveScene(scene);
    //     sceneManager->pause(); // Start simulation paused

    //     auto driver = std::make_shared<SimulationManager>();
    //     driver->addModule(viewer);
    //     driver->addModule(sceneManager);
    //     driver->setDesiredDt(0.001);

    //     driver->addModule(hapticManager);

    //     // Add mouse and keyboard controls to the viewer
    //     auto mouseControl = std::make_shared<MouseSceneControl>(viewer->getMouseDevice());
    //     mouseControl->setSceneManager(sceneManager);
    //     viewer->addControl(mouseControl);

    //     auto keyControl = std::make_shared<KeyboardSceneControl>(viewer->getKeyboardDevice());
    //     keyControl->setSceneManager(sceneManager);
    //     keyControl->setModuleDriver(driver);
    //     viewer->addControl(keyControl);

    //     driver->start();
    // }

    return 0;
}