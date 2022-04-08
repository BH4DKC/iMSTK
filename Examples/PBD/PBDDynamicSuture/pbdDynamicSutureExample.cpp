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
#include "imstkKeyboardSceneControl.h"
#include "imstkMeshIO.h"
#include "imstkMouseSceneControl.h"
#include "imstkPbdModel.h"
#include "imstkPbdObject.h"
#include "imstkPbdObjectCollision.h"
#include "imstkPbdObjectGrasping.h"
#include "imstkPointwiseMap.h"
#include "imstkRenderMaterial.h"
#include "imstkScene.h"
#include "imstkSceneManager.h"
#include "imstkSimulationManager.h"
#include "imstkSurfaceMesh.h"
#include "imstkTetrahedralMesh.h"
#include "imstkVisualModel.h"
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
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Distance, 1.0);
    pbdParams->enableConstraint(PbdModelConfig::ConstraintGenType::Volume, 1.0);
    pbdParams->m_doPartitioning   = false;
    pbdParams->m_uniformMassValue = 1.0;
    pbdParams->m_dt = 0.001;
    pbdParams->m_iterations = 5;
    pbdParams->m_gravity = Vec3d::Zero();
    pbdParams->m_viscousDampingCoeff = 0.03;

    // Setup the Model
    auto pbdModel = std::make_shared<PbdModel>();
    pbdModel->setModelGeometry(tetMesh);
    pbdModel->configure(pbdParams);

    // Setup the material
    auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
    // material->setDisplayMode(RenderMaterial::DisplayMode::Surface);
    // material->setShadingModel(RenderMaterial::ShadingModel::PBR);
    // material->addTexture(std::make_shared<Texture>(iMSTK_DATA_ROOT "/textures/fleshDiffuse.jpg",
    //         Texture::Type::Diffuse));
    // material->addTexture(std::make_shared<Texture>(iMSTK_DATA_ROOT "/textures/fleshNormal.jpg",
    //         Texture::Type::Normal));
    // material->addTexture(std::make_shared<Texture>(iMSTK_DATA_ROOT "/textures/fleshORM.jpg",
    //         Texture::Type::ORM));
    // material->setNormalStrength(0.3);

    // Add a visual model to render the surface of the tet mesh
    auto visualModel = std::make_shared<VisualModel>();
    visualModel->setGeometry(surfMesh);
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
    
    scene->getActiveCamera()->setPosition(-5.0, 0.05, 8.3);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.0, 0.0);
    scene->getActiveCamera()->setViewUp(0.85, 0.0, 0.5);

    // Load a tetrahedral mesh
    std::shared_ptr<TetrahedralMesh> tetMesh = MeshIO::read<TetrahedralMesh>(input.meshFileName);
    CHECK(tetMesh != nullptr) << "Could not read mesh from file.";

    // Scene object 1: Mesh with hold
    std::shared_ptr<PbdObject> tissueHole = createTissueHole(tetMesh);
    
    scene->addSceneObject(tissueHole);

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
        {

            auto mouseControl = std::make_shared<MouseSceneControl>(viewer->getMouseDevice());

            mouseControl->setSceneManager(sceneManager);
            viewer->addControl(mouseControl);


            auto keyControl = std::make_shared<KeyboardSceneControl>(viewer->getKeyboardDevice());
            keyControl->setSceneManager(sceneManager);
            keyControl->setModuleDriver(driver);
            viewer->addControl(keyControl);
        }

        // Simulate the tissue in real time
        connect<Event>(sceneManager, &SceneManager::postUpdate,
            [&](Event*)
            {
                // Simulate the cloth in real time
                tissueHole->getPbdModel()->getConfig()->m_dt = sceneManager->getDt();
            });


        driver->start();
    }

    return 0;
}