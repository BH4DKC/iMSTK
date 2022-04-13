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
// #include "NeedleEmbeddedCH.h"

#ifdef iMSTK_USE_OPENHAPTICS
#include "imstkHapticDeviceManager.h"
#include "imstkHapticDeviceClient.h"
#include "imstkRigidObjectController.h"
#endif

using namespace imstk;

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


///
/// \brief Create pbd string geometry
///
static std::shared_ptr<LineMesh>
makeStringGeometry(const Vec3d& pos, const Vec3d& dx, const int numVerts)
{
    // Create the geometry

    auto stringGeometry = std::make_shared<LineMesh>();

    auto verticesPtr = std::make_shared<VecDataArray<double, 3>>(numVerts);
    VecDataArray<double, 3>&          vertices = *verticesPtr.get();
    for (int i = 0; i < numVerts; i++)
    {
        vertices[i] = pos + dx * i;
    }

    // Add connectivity data
    auto segmentsPtr = std::make_shared<VecDataArray<int, 2>>();
    VecDataArray<int, 2>&          segments = *segmentsPtr.get();
    for (int i = 0; i < numVerts - 1; i++)
    {
        segments.push_back(Vec2i(i, i + 1));
    }

    stringGeometry->initialize(verticesPtr, segmentsPtr);
    return stringGeometry;
}

///
/// \brief Create pbd string object
///
static std::shared_ptr<PbdObject>
makePbdString(
    const std::string& name,
    const Vec3d& pos, const Vec3d& dir, const int numVerts,
    const double stringLength)
{

    auto stringObj = std::make_shared<PbdObject>(name);

    // Setup the Geometry
    const double              dx = stringLength / numVerts;
    std::shared_ptr<LineMesh> stringMesh = makeStringGeometry(pos, dir * dx, numVerts);

    // Setup the Parameters
    auto pbdParams = std::make_shared<PbdModelConfig>();
    // pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 100.0);
    // pbdParams->enableBendConstraint(100000.0, 1);
    // pbdParams->enableBendConstraint(100000.0, 2);

    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 1.0);
    pbdParams->enableBendConstraint(1000.0, 1);
    pbdParams->enableBendConstraint(1000.0, 2);
    pbdParams->m_fixedNodeIds     = { 0, 1 };
    pbdParams->m_uniformMassValue = 0.002 / numVerts; // grams
    pbdParams->m_gravity = Vec3d(0.0, -9.8, 0.0);
    pbdParams->m_dt      = 0.0005;                    // Overwritten for real time

    // Requires large amounts of iterations the longer, a different
    // solver would help
    pbdParams->m_iterations = 100;
    pbdParams->m_viscousDampingCoeff = 0.01;

    // Setup the Model
    auto pbdModel = std::make_shared<PbdModel>();
    pbdModel->setModelGeometry(stringMesh);
    pbdModel->configure(pbdParams);

    // Setup the material and VisualModel
    auto material = std::make_shared<RenderMaterial>();
    material->setBackFaceCulling(false);
    material->setColor(Color::Red);
    material->setLineWidth(2.0);
    material->setPointSize(6.0);
    material->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);

    auto visualModel = std::make_shared<VisualModel>();
    visualModel->setGeometry(stringMesh);
    visualModel->setRenderMaterial(material);

    // Setup the Object
    stringObj->addVisualModel(visualModel);
    stringObj->setPhysicsGeometry(stringMesh);
    stringObj->setCollidingGeometry(stringMesh);
    stringObj->setDynamicalModel(pbdModel);

    return stringObj;
}


static std::shared_ptr<SceneObject>
makeClampObj(std::string name)
{
    auto surfMesh =
        MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "/Surgical Instruments/Clamps/Gregory Suture Clamp/gregory_suture_clamp.obj");

    surfMesh->scale(5.0, Geometry::TransformType::ApplyToData);


    auto toolObj = std::make_shared<SceneObject>(name);
    toolObj->setVisualGeometry(surfMesh);
    auto renderMaterial = std::make_shared<RenderMaterial>();
    renderMaterial->setColor(Color::LightGray);
    renderMaterial->setShadingModel(RenderMaterial::ShadingModel::PBR);
    renderMaterial->setRoughness(0.5);
    renderMaterial->setMetalness(1.0);
    toolObj->getVisualModel(0)->setRenderMaterial(renderMaterial);

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

    // Construct the scene
    auto scene = std::make_shared<Scene>("DynamicSuture");
    
    scene->getActiveCamera()->setPosition(0.0, 0.0, 7.7);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.0, -0.5);
    scene->getActiveCamera()->setViewUp(0.0, 1.0, 0.0);


    auto light = std::make_shared<DirectionalLight>();
    light->setFocalPoint(Vec3d(5.0, -8.0, -5.0));
    light->setIntensity(1.0);
    scene->addLight("Light", light);

    // Load a tetrahedral mesh
    std::shared_ptr<TetrahedralMesh> tetMesh = MeshIO::read<TetrahedralMesh>(input.meshFileName);
    CHECK(tetMesh != nullptr) << "Could not read mesh from file.";


    // Mesh with hole for suturing
    std::shared_ptr<PbdObject> tissueHole = createTissueHole(tetMesh);
    scene->addSceneObject(tissueHole);

    // Create arced needle
    auto needleObj = std::make_shared<NeedleObject>();
    needleObj->setForceThreshold(2.0);
    scene->addSceneObject(needleObj);

    // Create the suture pbd-based string
    const double               stringLength      = 4.0;
    const int                  stringVertexCount = 30;
    std::shared_ptr<PbdObject> sutureThreadObj   =
        makePbdString("SutureThread", Vec3d(0.0, 0.0, 0.018), Vec3d(0.0, 0.0, 1.0),
            stringVertexCount, stringLength);
    scene->addSceneObject(sutureThreadObj);

    // Create clamps that follow the needle around
    std::shared_ptr<SceneObject> toolObj = makeClampObj("Clamps");
    scene->addSceneObject(toolObj);

    // Create ghost clamps to show real position of hand under virtual coupling
    std::shared_ptr<SceneObject> ghostToolObj = makeClampObj("GhostClamps");
    ghostToolObj->getVisualModel(0)->getRenderMaterial()->setColor(Color::Orange);
    scene->addSceneObject(ghostToolObj);

    // Add point based collision between the tissue & suture thread
    // auto interaction = std::make_shared<PbdObjectCollision>(sutureThreadObj, tissueHole, "MeshToMeshBruteForceCD");
    // interaction->setFriction(0.0);
    // scene->addInteraction(interaction);

    // WARNING: Must be modified, shouldnt currently work
    // Add needle constraining behaviour between the tissue & arc needle
    // auto needleInteraction = std::make_shared<NeedleInteraction>(tissueHole, needleObj);
    // scene->addInteraction(needleInteraction);
    




    // scene->getConfig()->writeTaskGraph = true;
    

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

        auto hapticManager = std::make_shared<HapticDeviceManager>();
        hapticManager->setSleepDelay(0.01); // Delay for 1ms (haptics thread is limited to max 1000hz)
        
        std::shared_ptr<HapticDeviceClient> deviceClient = hapticManager->makeDeviceClient();
        driver->addModule(hapticManager);

        auto hapController = std::make_shared<RigidObjectController>(needleObj, deviceClient);
        hapController->setTranslationScaling(0.05);
        hapController->setLinearKs(10000.0);
        hapController->setAngularKs(1000000.0);
        //hapController->setAngularKs(0.0);
        hapController->setUseCritDamping(true);
        hapController->setForceScaling(0.001);
        hapController->setSmoothingKernelSize(10);
        hapController->setUseForceSmoothening(true);
        scene->addController(hapController);

        
        // Update the timesteps for real time
        connect<Event>(sceneManager, &SceneManager::preUpdate,
            [&](Event*)
            {
                needleObj->getRigidBodyModel2()->getConfig()->m_dt = sceneManager->getDt();
                sutureThreadObj->getPbdModel()->getConfig()->m_dt  = sceneManager->getDt();
            });
        
        // Constrain the first two vertices of the string to the needle
        connect<Event>(sceneManager, &SceneManager::postUpdate,
            [&](Event*)
            {
                auto needleLineMesh = std::dynamic_pointer_cast<LineMesh>(needleObj->getPhysicsGeometry());
                auto sutureLineMesh = std::dynamic_pointer_cast<LineMesh>(sutureThreadObj->getPhysicsGeometry());
                (*sutureLineMesh->getVertexPositions())[1] = (*needleLineMesh->getVertexPositions())[0];
                (*sutureLineMesh->getVertexPositions())[0] = (*needleLineMesh->getVertexPositions())[1];
            });
        
        // Transform the clamps relative to the needle
        const Vec3d clampOffset = Vec3d(-0.009, 0.01, 0.001);
        connect<Event>(sceneManager, &SceneManager::postUpdate,
            [&](Event*)
            {
                toolObj->getVisualGeometry()->setTransform(
                    needleObj->getVisualGeometry()->getTransform() *
                    mat4dTranslate(clampOffset) *
                    mat4dRotation(Rotd(PI, Vec3d(0.0, 1.0, 0.0))));
                toolObj->getVisualGeometry()->postModified();
            });
        
        // Transform the ghost tool clamps to show the real tool location
        connect<Event>(sceneManager, &SceneManager::postUpdate,
            [&](Event*)
            {
                ghostToolObj->getVisualGeometry()->setTransform(
                    mat4dTranslate(hapController->getPosition()) * mat4dRotation(hapController->getOrientation()) *
                    mat4dTranslate(clampOffset) *
                    mat4dRotation(Rotd(PI, Vec3d(0.0, 1.0, 0.0))));
                ghostToolObj->getVisualGeometry()->updatePostTransformData();
                ghostToolObj->getVisualGeometry()->postModified();
                ghostToolObj->getVisualModel(0)->getRenderMaterial()->setOpacity(std::min(1.0, hapController->getDeviceForce().norm() / 5.0));
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

        scene->getConfig()->writeTaskGraph = true;

        driver->start();
    }


    return 0;
}