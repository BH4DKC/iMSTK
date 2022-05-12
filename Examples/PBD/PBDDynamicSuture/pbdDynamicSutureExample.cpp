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
#include "imstkSphere.h"
#include "imstkTetrahedralMesh.h"
#include "imstkVisualModel.h"
#include "imstkVRPNDeviceClient.h"
#include "imstkVRPNDeviceManager.h"
#include "imstkVTKViewer.h"

#include "NeedleInteraction.h"
#include "NeedleObject.h"
#include "NeedlePbdCH.h"

#include "imstkNew.h"
#include "imstkImageData.h"


// #include "NeedleEmbeddedCH.h"

#ifdef iMSTK_USE_OPENHAPTICS
#include "imstkHapticDeviceManager.h"
#include "imstkHapticDeviceClient.h"
#include "imstkRigidObjectController.h"
#endif

using namespace imstk;

Vec3d NeedlePbdCH::debugPt;
Vec3d NeedlePbdCH::debugPt2;

struct Input
{
    std::string meshFileName;
};

Input input;

///
/// \brief Spherically project the texture coordinates
///
static void
setSphereTexCoords(std::shared_ptr<SurfaceMesh> surfMesh, const double uvScale)
{
    Vec3d min, max;
    surfMesh->computeBoundingBox(min, max);
    const Vec3d size = max - min;
    Vec3d center = (max + min) * 0.5;

    // center[1] += -10.0;

    const double radius = (size * 0.5).norm();

    imstkNew<VecDataArray<float, 2>> uvCoordsPtr(surfMesh->getNumVertices());

    // auto uvCoordsPtr = std::make_shared<VecDataArray<float, 2>>(surfMesh->getNumVertices());
    
    VecDataArray<float, 2>& uvCoords = *uvCoordsPtr.get();
    for (int i = 0; i < surfMesh->getNumVertices(); i++)
    {
        Vec3d vertex = surfMesh->getVertexPosition(i) - center;

        // Compute phi and theta on the sphere
        const double theta = asin(vertex[0] / radius);
        const double phi = atan2(vertex[1], vertex[2]);
        uvCoords[i] = Vec2f(phi / (PI * 2.0) + 0.5, theta / (PI * 2.0) + 0.5) * uvScale;
    }
    surfMesh->setVertexTCoords("tcoords", uvCoordsPtr);
}

// Create tissue object to stitch
std::shared_ptr<PbdObject>
createTissueHole(std::shared_ptr<TetrahedralMesh> tetMesh)
{

    std::shared_ptr<SurfaceMesh> surfMesh = tetMesh->extractSurfaceMesh();


    auto pbdObject = std::make_shared<PbdObject>("meshHole");
    auto pbdParams = std::make_shared<PbdModelConfig>();

    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 10.0);
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Volume, 10.0);
    pbdParams->m_doPartitioning = false;
    pbdParams->m_uniformMassValue = 0.01;
    pbdParams->m_gravity = Vec3d(0.0, 0.0, -0.1);
    pbdParams->m_dt = 0.01;
    pbdParams->m_iterations = 5;
    pbdParams->m_viscousDampingCoeff = 0.01;

    // Fix the borders
    for (int vert_id = 0; vert_id < surfMesh->getNumVertices(); vert_id++)
    {
        auto position = tetMesh->getVertexPosition(vert_id);
        if (std::fabs(1.40984 - std::fabs(position(1))) <= 1E-4)
        {
            pbdParams->m_fixedNodeIds.push_back(vert_id);
        }
    }

    tetMesh->rotate(Vec3d(0.0, 0.0, 1.0), -PI_2, Geometry::TransformType::ApplyToData);
    tetMesh->rotate(Vec3d(1.0, 0.0, 0.0), -PI_2 / 1.0, Geometry::TransformType::ApplyToData);

    surfMesh->rotate(Vec3d(0.0, 0.0, 1.0), -PI_2, Geometry::TransformType::ApplyToData);
    surfMesh->rotate(Vec3d(1.0, 0.0, 0.0), -PI_2 / 1.0, Geometry::TransformType::ApplyToData);

    tetMesh->scale(0.03, Geometry::TransformType::ApplyToData);
    surfMesh->scale(0.03, Geometry::TransformType::ApplyToData);



    setSphereTexCoords(surfMesh, 10);

    surfMesh->computeVertexNormals();
    surfMesh->computeTriangleTangents();
    surfMesh->computeVertexTangents();

    // Setup the Model
    auto pbdModel = std::make_shared<PbdModel>();
    pbdModel->setModelGeometry(tetMesh);
    pbdModel->configure(pbdParams);

    // Setup the material
    /*auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::WireframeSurface);*/

    // Setup the material
    auto material = std::make_shared<RenderMaterial>();
    material->setShadingModel(RenderMaterial::ShadingModel::PBR);
    auto diffuseTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshDiffuse.jpg");
    material->addTexture(std::make_shared<Texture>(diffuseTex, Texture::Type::Diffuse));
    auto normalTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshNormal.jpg");
    material->addTexture(std::make_shared<Texture>(normalTex, Texture::Type::Normal));
    auto ormTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshORM.jpg");
    material->addTexture(std::make_shared<Texture>(ormTex, Texture::Type::ORM));
    material->setNormalStrength(0.3);

    // Add a visual model to render the surface of the tet mesh
    auto visualModel = std::make_shared<VisualModel>();
    visualModel->setGeometry(surfMesh);
    visualModel->setRenderMaterial(material);
    pbdObject->addVisualModel(visualModel);

    MeshIO::write(surfMesh, "test.vtk");

    // Setup the Object
    pbdObject->setPhysicsGeometry(tetMesh);
    pbdObject->setCollidingGeometry(surfMesh);
    pbdObject->setPhysicsToCollidingMap(std::make_shared<PointwiseMap>(tetMesh, surfMesh));
    pbdObject->setDynamicalModel(pbdModel);

    return pbdObject;
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
    pbdParams->m_viscousDampingCoeff = 0.001;

    // Setup the Model
    auto pbdModel = std::make_shared<PbdModel>();
    pbdModel->setModelGeometry(triMesh);
    pbdModel->configure(pbdParams);

    // Setup the material
    auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::WireframeSurface);
    material->setBackFaceCulling(false);


    // Add a visual model to render the surface of the tet mesh
    auto visualModel = std::make_shared<VisualModel>();
    visualModel->setGeometry(triMesh);
    visualModel->setRenderMaterial(material);
    pbdObject->addVisualModel(visualModel);

    // Add a visual model to render the normals of the surface
    auto normalsVisualModel = std::make_shared<VisualModel>();
    normalsVisualModel->setGeometry(triMesh);
    normalsVisualModel->getRenderMaterial()->setDisplayMode(RenderMaterial::DisplayMode::SurfaceNormals);
    normalsVisualModel->getRenderMaterial()->setPointSize(0.025);
    pbdObject->addVisualModel(normalsVisualModel);

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

    input.meshFileName = iMSTK_DATA_ROOT "Tissues/tissue_hole.vtk";
    // input.meshFileName = iMSTK_DATA_ROOT "Tissues/tissue_hole.obj";

    // Construct the scene
    auto scene = std::make_shared<Scene>("DynamicSuture");

    scene->getActiveCamera()->setPosition(0.0, 0.06, 0.24);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.005, 0.05);
    scene->getActiveCamera()->setViewUp(-0.01, 1.0, -0.3);

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

    // Make Triangle
   /* auto pbdTriangle = createPbdTriangle();
    scene->addSceneObject(pbdTriangle);*/

    // Create arced needle
    auto needleObj = std::make_shared<NeedleObject>();
    needleObj->setForceThreshold(0.0001);
    scene->addSceneObject(needleObj);

    // Add needle constraining behaviour between the tissue & arc needle
    auto needleInteraction = std::make_shared<NeedleInteraction>(tissueHole, needleObj);
    scene->addInteraction(needleInteraction);

    auto sphere = std::make_shared<Sphere>();
    sphere->setRadius(0.002);

    auto sphereModel = std::make_shared<VisualModel>();
    sphereModel->setGeometry(sphere);
    sphereModel->getRenderMaterial()->setColor(Color::Green);

    auto sphereObj = std::make_shared<SceneObject>("SphereObj");
    sphereObj->addVisualModel(sphereModel);
    scene->addSceneObject(sphereObj);


    auto sphere2 = std::make_shared<Sphere>();
    sphere2->setRadius(0.002);

    auto sphereModel2 = std::make_shared<VisualModel>();
    sphereModel2->setGeometry(sphere2);
    sphereModel2->getRenderMaterial()->setColor(Color::Red);

    auto sphereObj2 = std::make_shared<SceneObject>("SphereObj2");
    sphereObj2->addVisualModel(sphereModel2);
    scene->addSceneObject(sphereObj2);


    scene->getConfig()->writeTaskGraph = true;

    // Run the simulation

    // Note: Collision data can come from CD consistently
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
                // pbdTriangle->getPbdModel()->getConfig()->m_dt      = sceneManager->getDt();
                tissueHole->getPbdModel()->getConfig()->m_dt = sceneManager->getDt();
            });

        // Move the sphere to the contact point
        connect<Event>(sceneManager, &SceneManager::preUpdate,
            [&](Event*)
            {
                sphere->setPosition(NeedlePbdCH::debugPt); // current intersection
                sphere2->setPosition(NeedlePbdCH::debugPt2); // Puncture point

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