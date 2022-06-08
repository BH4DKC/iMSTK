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
#include "imstkGeometryUtilities.h"
#include "imstkPointLight.h"
#include "imstkIsometricMap.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
#include "imstkLineMesh.h"
#include "imstkMeshIO.h"
// #include "imstkMeshToMeshBruteForceCD.h"
#include "imstkClosedSurfaceMeshToMeshCD.h"
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
// std::vector<Vec3d> NeedlePbdCH::debugPt;
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

    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 5.0);
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Volume, 100.0);
    pbdParams->m_doPartitioning = false;
    pbdParams->m_uniformMassValue = 0.01;
    pbdParams->m_gravity = Vec3d(0.0, 0.0, -0.1);
    pbdParams->m_dt = 0.01;
    pbdParams->m_iterations = 5;
    pbdParams->m_viscousDampingCoeff = 0.3;

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

    tetMesh->scale(0.015, Geometry::TransformType::ApplyToData); // 0.015
    surfMesh->scale(0.015, Geometry::TransformType::ApplyToData);


    setSphereTexCoords(surfMesh, 10);

    surfMesh->computeVertexNormals();
    surfMesh->computeTriangleTangents();
    surfMesh->computeVertexTangents();

    // Setup the Model
    auto pbdModel = std::make_shared<PbdModel>();
    pbdModel->setModelGeometry(tetMesh);
    pbdModel->configure(pbdParams);

    // Setup the material
    auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::WireframeSurface);

    // Setup the material
    /*auto material = std::make_shared<RenderMaterial>();
    material->setShadingModel(RenderMaterial::ShadingModel::PBR);
    auto diffuseTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshDiffuse.jpg");
    material->addTexture(std::make_shared<Texture>(diffuseTex, Texture::Type::Diffuse));
    auto normalTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshNormal.jpg");
    material->addTexture(std::make_shared<Texture>(normalTex, Texture::Type::Normal));
    auto ormTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshORM.jpg");
    material->addTexture(std::make_shared<Texture>(ormTex, Texture::Type::ORM));
    material->setNormalStrength(0.3);*/

    // Add a visual model to render the surface of the tet mesh
    auto visualModel = std::make_shared<VisualModel>();
    visualModel->setGeometry(surfMesh);
    visualModel->setRenderMaterial(material);
    pbdObject->addVisualModel(visualModel);

    // MeshIO::write(surfMesh, "test.vtk");

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
/// \brief Create pbd string object
///
static std::shared_ptr<PbdObject>
makePbdString(
    const std::string& name,
    const Vec3d& pos, const Vec3d& dir, const int numVerts,
    const double stringLength)
{
    imstkNew<PbdObject> stringObj(name);

    // Setup the Geometry
    std::shared_ptr<LineMesh> stringMesh =
        GeometryUtils::toLineGrid(pos, dir, stringLength, numVerts);

    // Setup the Parameters
    imstkNew<PbdModelConfig> pbdParams;
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 100);
    pbdParams->enableBendConstraint(1.0, 1);
    // pbdParams->enableBendConstraint(10.0, 2); // 100000.0
    pbdParams->m_fixedNodeIds = { 0, 1 };
    pbdParams->m_uniformMassValue = 0.001 / numVerts; // 0.002 / numVerts; // grams
    pbdParams->m_gravity = Vec3d(0.0, 0.0, 0.0);
    pbdParams->m_dt = 0.0005; // Overwritten for real time

    // Requires large amounts of iterations the longer, a different
    // solver would help
    pbdParams->m_iterations = 70;
    pbdParams->m_viscousDampingCoeff = 0.03;

    // Setup the Model
    imstkNew<PbdModel> pbdModel;
    pbdModel->setModelGeometry(stringMesh);
    pbdModel->configure(pbdParams);

    // Setup the VisualModel
    imstkNew<RenderMaterial> material;
    material->setBackFaceCulling(false);
    material->setColor(Color::Red);
    material->setLineWidth(2.0);
    material->setPointSize(18.0);
    material->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
    // material->setDisplayMode(RenderMaterial::DisplayMode::Points);

    imstkNew<VisualModel> visualModel;
    visualModel->setGeometry(stringMesh);
    visualModel->setRenderMaterial(material);

    // Setup the Object
    stringObj->addVisualModel(visualModel);
    stringObj->setPhysicsGeometry(stringMesh);
    stringObj->setCollidingGeometry(stringMesh);
    stringObj->setDynamicalModel(pbdModel);

    return stringObj;
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

    // add new mesh overaying physics mesh
    /*auto surfaceMesh = MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "Tissues/tissue_hole_test.obj");

    surfaceMesh->rotate(Vec3d(0.0, 0.0, 1.0), -PI_2, Geometry::TransformType::ApplyToData);
    surfaceMesh->rotate(Vec3d(1.0, 0.0, 0.0), -PI_2 / 1.0, Geometry::TransformType::ApplyToData);

    surfaceMesh->scale(0.03, Geometry::TransformType::ApplyToData);
    

    auto material2 = std::make_shared<RenderMaterial>();
    material2->setDisplayMode(RenderMaterial::DisplayMode::Surface);
    material2->setShadingModel(RenderMaterial::ShadingModel::PBR);

    material2->setShadingModel(RenderMaterial::ShadingModel::PBR);
    auto diffuseTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshDiffuse.jpg");
    material2->addTexture(std::make_shared<Texture>(diffuseTex, Texture::Type::Diffuse));
    auto normalTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshNormal.jpg");
    material2->addTexture(std::make_shared<Texture>(normalTex, Texture::Type::Normal));
    auto ormTex = MeshIO::read<ImageData>(iMSTK_DATA_ROOT "/textures/fleshORM.jpg");
    material2->addTexture(std::make_shared<Texture>(ormTex, Texture::Type::ORM));

    material2->setRecomputeVertexNormals(false);

    auto surfaceMeshModel = std::make_shared<VisualModel>();
    surfaceMeshModel->setGeometry(surfaceMesh);
    surfaceMeshModel->setRenderMaterial(material2);

    auto sceneMesh = std::make_shared<SceneObject>("TestMesh");
    sceneMesh->addVisualModel(surfaceMeshModel);*/
   

    // Construct the scene
    auto scene = std::make_shared<Scene>("DynamicSuture");

    // scene->addSceneObject(sceneMesh);

    scene->getActiveCamera()->setPosition(0.0, 0.04, 0.09);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.02, 0.05);
    scene->getActiveCamera()->setViewUp(0.001, 1.0, -0.4);

    auto light = std::make_shared<DirectionalLight>();
    light->setFocalPoint(Vec3d(5.0, -8.0, -5.0));
    // light->setFocalPoint(Vec3d(0.0, -20.0, -20.0));
    // light->setDirection(Vec3d(0.0, 0.005, 0.05));
    light->setIntensity(1.0);
    scene->addLight("Light", light);


    auto ptLight = std::make_shared<PointLight>();
    ptLight->setPosition(0.0, 0.06, 0.24);
    ptLight->setIntensity(2.0);


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

    // Create the suture pbd-based string
    const double               stringLength = 0.105;
    const int                  stringVertexCount = 35;
    std::shared_ptr<PbdObject> sutureThreadObj =
        makePbdString("SutureThread", Vec3d(0.0, 0.0, 0.018), Vec3d(0.0, 0.0, 1.0),
            stringVertexCount, stringLength);
    scene->addSceneObject(sutureThreadObj);

    // Add needle constraining behaviour between the tissue & arc needle/thread
    auto needleInteraction = std::make_shared<NeedleInteraction>(tissueHole, needleObj, sutureThreadObj);
    scene->addInteraction(needleInteraction);

    // Add point based collision between the tissue & suture thread
    //auto interaction = std::make_shared<PbdObjectCollision>(sutureThreadObj, tissueHole, "MeshToMeshBruteForceCD");
    //interaction->setFriction(0.0);
    //scene->addInteraction(interaction);

    // Spheres for debugging
    auto sphere = std::make_shared<Sphere>();
    sphere->setRadius(0.001);

    auto sphereModel = std::make_shared<VisualModel>();
    sphereModel->setGeometry(sphere);
    sphereModel->getRenderMaterial()->setColor(Color::Green);

    auto sphereObj = std::make_shared<SceneObject>("SphereObj");
    sphereObj->addVisualModel(sphereModel);
    scene->addSceneObject(sphereObj);


    auto sphere2 = std::make_shared<Sphere>();
    sphere2->setRadius(0.001);

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
                // sphere2->setPosition(NeedlePbdCH::debugPt[1]);
                sphere2->setPosition(NeedlePbdCH::debugPt2); // Puncture point

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

        connect<KeyEvent>(viewer->getKeyboardDevice(), &KeyboardDeviceClient::keyPress,
            [&](KeyEvent* e)
            {
                // Perform stitch
                if (e->m_key == 's')
                {
                    needleInteraction->stitch();
                }
            });

        // scene->getConfig()->writeTaskGraph = true;

        driver->start();
    }

    return 0;
}