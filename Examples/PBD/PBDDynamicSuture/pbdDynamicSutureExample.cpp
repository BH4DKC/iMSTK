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
#include "imstkDirectionalLight.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
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



using namespace imstk;

std::shared_ptr<PbdObject> createTissueHole(std::shared_ptr<TetrahedralMesh> tetMesh);

struct Input
{
    std::string meshFileName;
};

Input input;

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

static std::shared_ptr<RigidObject2>
makeCapsuleToolObj()
{
    // Set up rigid body model
    std::shared_ptr<RigidBodyModel2> rbdModel = std::make_shared<RigidBodyModel2>();
    rbdModel->getConfig()->m_gravity = Vec3d::Zero();
    rbdModel->getConfig()->m_maxNumIterations       = 8;
    rbdModel->getConfig()->m_velocityDamping        = 1.0;
    rbdModel->getConfig()->m_angularVelocityDamping = 1.0;
    rbdModel->getConfig()->m_maxNumConstraints      = 40;

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
/// \brief This example demonstrates suturing of a hole in a tissue
///

int
main()
{
    // Setup logger (write to file and stdout)
    Logger::startLogger();

    input.meshFileName = iMSTK_DATA_ROOT "Tissues/tissue_hole.vtk";

    /// \ IP address of the server.
    const std::string serverIP = "localhost";

    /// \ the default VRPN port 38833
    const int serverPort = 3883;

    // VRPN Server for 3D mouse control
    auto server = std::make_shared<VRPNDeviceManager>(serverIP, serverPort);
    auto client = server->makeDeviceClient("tracker0", VRPNTracker);

    // Construct the scene
    auto scene = std::make_shared<Scene>("DynamicSuture");
    
    scene->getActiveCamera()->setPosition(0.0, 0.0, 10.0);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.0, 1.0);
    scene->getActiveCamera()->setViewUp(0.0, 1.0, 0.0);

    // Load a tetrahedral mesh
    std::shared_ptr<TetrahedralMesh> tetMesh = MeshIO::read<TetrahedralMesh>(input.meshFileName);
    CHECK(tetMesh != nullptr) << "Could not read mesh from file.";

    // Scene object 1: Mesh with hole
    std::shared_ptr<PbdObject> tissueHole = createTissueHole(tetMesh);
    scene->addSceneObject(tissueHole);

    std::shared_ptr<RigidObject2> toolObj = makeCapsuleToolObj();
    scene->addSceneObject(toolObj);


    // Add 3D mouse Controller for capsule
    auto controller = std::make_shared<SceneObjectController>(toolObj, client);
    controller->setTranslationScaling(1.0);
    scene->addController(controller);


    // Light
    auto light = std::make_shared<DirectionalLight>();
    light->setFocalPoint(Vec3d(0.0, -1.0, -1.0));
    light->setIntensity(1.0);
    scene->addLight("light", light);
    

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

        auto driver = std::make_shared<SimulationManager>();
        driver->addModule(viewer);
        driver->addModule(sceneManager);
        driver->setDesiredDt(0.001);

        // Add mouse and keyboard controls to the viewer
        auto mouseControl = std::make_shared<MouseSceneControl>(viewer->getMouseDevice());
        mouseControl->setSceneManager(sceneManager);
        viewer->addControl(mouseControl);

        auto keyControl = std::make_shared<KeyboardSceneControl>(viewer->getKeyboardDevice());
        keyControl->setSceneManager(sceneManager);
        keyControl->setModuleDriver(driver);
        viewer->addControl(keyControl);

        // Simulate the tissue in real time-
        // connect<Event>(sceneManager, &SceneManager::postUpdate, [&](Event*)
        //     {
        //         tissueHole->getPbdModel()->getConfig()->m_dt = sceneManager->getDt();
        //         // toolObj->getRigidBodyModel2()->getConfig()->m_dt = sceneManager->getDt();
        //     });

        // connect<Event>(sceneManager, &SceneManager::postUpdate,
        //     [&](Event*)
        //     {
        //         const Vec2d mousePos = viewer->getMouseDevice()->getPos();
        //         const Vec3d worldPos = Vec3d(mousePos[0] - 0.5, mousePos[1] - 0.5, 0.0) * 10.0;

        //         const Vec3d fS = (worldPos - toolObj->getRigidBody()->getPosition()) * 100000.0; // Spring force
        //         const Vec3d fD = -toolObj->getRigidBody()->getVelocity() * 100.0;                // Spring damping

        //         (*toolObj->getRigidBody()->m_force) += (fS + fD);

        //     });


        driver->start();
    }

    return 0;
}