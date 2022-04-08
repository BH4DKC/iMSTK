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
#include "imstkMeshToMeshBruteForceCD.h"
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
// #include "NeedleEmbeddedCH.h"

#ifdef iMSTK_USE_OPENHAPTICS
#include "imstkHapticDeviceManager.h"
#include "imstkHapticDeviceClient.h"
#include "imstkRigidObjectController.h"
#endif

using namespace imstk;

std::shared_ptr<SurfaceMesh>
createTriangle()
{
    auto triangleMesh = std::make_shared<SurfaceMesh>();

    VecDataArray<double, 3> triangleVertices(3);
    triangleVertices[0] = Vec3d(-0.1, 0.0, 0.0);
    triangleVertices[1] = Vec3d(0.0, 0.0, 0.1);
    triangleVertices[2] = Vec3d(0.1, 0.0, 0.0);
    VecDataArray<int, 3> triangleIndices(1);
    triangleIndices[0] = Vec3i(0, 1, 2);
    triangleMesh->initialize(
        std::make_shared<VecDataArray<double, 3>>(triangleVertices),
        std::make_shared<VecDataArray<int, 3>>(triangleIndices));

    return triangleMesh;
}

// Create tissue object to stitch
static std::shared_ptr<PbdObject>
createPbdTriangle()
{
    std::shared_ptr<SurfaceMesh> triMesh = createTriangle();

    auto pbdObject = std::make_shared<PbdObject>("pbdTriangle");
    auto pbdParams = std::make_shared<PbdModelConfig>();

    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 1.0);
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Dihedral, 1.0);
    pbdParams->m_doPartitioning   = false;
    pbdParams->m_uniformMassValue = 0.1;
    pbdParams->m_gravity    = Vec3d(0.0, 0.0, 0.0);
    pbdParams->m_dt         = 0.005;
    pbdParams->m_iterations = 5;
    pbdParams->m_viscousDampingCoeff = 0.00;

    // Setup the Model
    auto pbdModel = std::make_shared<PbdModel>();
    pbdModel->setModelGeometry(triMesh);
    pbdModel->configure(pbdParams);

    // Setup the material
    auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::WireframeSurface);

    // Add a visual model to render the surface of the tet mesh
    auto visualModel = std::make_shared<VisualModel>();
    visualModel->setGeometry(triMesh);
    visualModel->setRenderMaterial(material);
    pbdObject->addVisualModel(visualModel);

    // Setup the Object
    pbdObject->setPhysicsGeometry(triMesh);
    pbdObject->setCollidingGeometry(triMesh);
    pbdObject->setDynamicalModel(pbdModel);

    return pbdObject;
}

///
/// \brief This example demonstrates suturing of a hole in a tissue
///
int
main()
{
    // Setup logger (write to file and stdout)
    Logger::startLogger();

    // Construct the scene
    auto scene = std::make_shared<Scene>("DynamicSuture");

    scene->getActiveCamera()->setPosition(0.0, 0.06, 0.24);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.005, 0.05);
    scene->getActiveCamera()->setViewUp(-0.01, 1.0, -0.3);

    auto light = std::make_shared<DirectionalLight>();
    light->setFocalPoint(Vec3d(5.0, -8.0, -5.0));
    light->setIntensity(1.0);
    scene->addLight("Light", light);

    // Make Triangle
    auto pbdTriangle = createPbdTriangle();
    scene->addSceneObject(pbdTriangle);

    // Create arced needle
    auto needleObj = std::make_shared<NeedleObject>();
    needleObj->setForceThreshold(0.0001);
    scene->addSceneObject(needleObj);

    // Add needle constraining behaviour between the tissue & arc needle
    auto needleInteraction = std::make_shared<NeedleInteraction>(pbdTriangle, needleObj);
    scene->addInteraction(needleInteraction);

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
        sceneManager->pause();         // Start simulation paused

        // Setup a simulation manager to manage renders & scene updates
        auto driver = std::make_shared<SimulationManager>();
        driver->addModule(viewer);
        driver->addModule(sceneManager);
        driver->setDesiredDt(0.001);         // 1ms, 1000hz

        auto hapticManager = std::make_shared<HapticDeviceManager>();
        hapticManager->setSleepDelay(0.01);         // Delay for 1ms (haptics thread is limited to max 1000hz)

        std::shared_ptr<HapticDeviceClient> deviceClient = hapticManager->makeDeviceClient();
        driver->addModule(hapticManager);

        auto hapController = std::make_shared<RigidObjectController>(needleObj, deviceClient);
        hapController->setTranslationScaling(0.002);
        hapController->setLinearKs(10000.0);
        hapController->setAngularKs(1000000.0);
        //hapController->setAngularKs(0.0);
        hapController->setUseCritDamping(true);
        hapController->setForceScaling(0.001);
        hapController->setSmoothingKernelSize(10);
        hapController->setUseForceSmoothening(true);
        scene->addController(hapController);

        // Update the needle opbject for real time
        connect<Event>(sceneManager, &SceneManager::preUpdate,
            [&](Event*)
            {
                needleObj->getRigidBodyModel2()->getConfig()->m_dt = sceneManager->getDt();
                pbdTriangle->getPbdModel()->getConfig()->m_dt = sceneManager->getDt();
            });

        // Add mouse and keyboard controls to the viewer
        {
            auto mouseControl = std::make_shared<MouseSceneControl>(viewer->getMouseDevice());
            mouseControl->setSceneManager(sceneManager);
            viewer->addControl(mouseControl);

            auto keyControl = std::make_shared<KeyboardSceneControl>(viewer->getKeyboardDevice());
            keyControl->setSceneManager(sceneManager);
            keyControl->setModuleDriver(driver);
            viewer->addControl(keyControl);
        }

        // scene->getConfig()->writeTaskGraph = true;

        driver->start();
    }

    return 0;
}