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

#include "imstkSimulationManager.h"
#include "imstkRigidObject.h"
#include "imstkRigidBodyModel.h"
#include "imstkSceneObject.h"
#include "imstkTetrahedralMesh.h"
#include "imstkCube.h"
#include "imstkPlane.h"
#include "imstkSphere.h"
#include "imstkMeshIO.h"
#include "imstkIsometricMap.h"

#include "imstkSimulationManager.h"
#include "imstkVirtualCouplingCH.h"
#include "imstkHDAPIDeviceServer.h"
#include "imstkHDAPIDeviceClient.h"
#include "imstkSceneObjectController.h"
#include "imstkPlane.h"
#include "imstkSphere.h"
#include "imstkRigidObject.h"
#include "imstkRigidBodyModel.h"
#include "imstkMeshIO.h"
#include "imstkCube.h"
#include "imstkIsometricMap.h"

// global variables
const std::string phantomOmni1Name = "Default Device";

using namespace imstk;

std::shared_ptr<imstk::RigidObject>
addMeshRigidObject(std::string& name, std::shared_ptr<Scene> scene, Vec3d pos)
{
    // create cube object
    auto meshObj = std::make_shared<RigidObject>(name);

    // Load a tetrahedral mesh
    auto tetMesh = imstk::MeshIO::read(iMSTK_DATA_ROOT "/asianDragon/asianDragon.veg");
    if (!tetMesh)
    {
        LOG(FATAL) << "Could not read mesh from file.";
    }

    // Extract the surface mesh
    auto surfMesh   = std::make_shared<SurfaceMesh>();
    auto volTetMesh = std::dynamic_pointer_cast<TetrahedralMesh>(tetMesh);
    if (!volTetMesh)
    {
        LOG(FATAL) << "Dynamic pointer cast from PointSet to TetrahedralMesh failed!";
    }
    volTetMesh->scale(15., Geometry::TransformType::ApplyToData);
    volTetMesh->translate(pos, Geometry::TransformType::ApplyToData);
    volTetMesh->extractSurfaceMesh(surfMesh, true);

    // add visual model
    auto renderModel = std::make_shared<VisualModel>(surfMesh);
    auto mat         = std::make_shared<RenderMaterial>();
    mat->setDisplayMode(RenderMaterial::WIREFRAME_SURFACE);
    mat->setLineWidth(2.);
    mat->setColor(Color::Green);
    renderModel->setRenderMaterial(mat);
    meshObj->addVisualModel(renderModel);

    // add dynamic model
    auto rigidModel = std::make_shared<RigidBodyModel>();
    auto rigidProp  = std::make_shared<RigidBodyPropertyDesc>();
    rigidModel->configure(surfMesh, rigidProp, RigidBodyType::Kinematic);
    meshObj->setPhysicsGeometry(surfMesh);
    meshObj->setDynamicalModel(rigidModel);

    // add cube to scene
    scene->addSceneObject(meshObj);
    return meshObj;
}

std::shared_ptr<imstk::RigidObject>
addCubeRigidObject(std::string& name, std::shared_ptr<Scene> scene, Vec3d pos, const bool isStatic = false)
{
    // create cube object
    auto cubeObj = std::make_shared<RigidObject>(name);

    // Create Cube object
    auto cubeGeom = std::make_shared<Cube>();
    cubeGeom->setWidth(20.);
    cubeGeom->translate(pos);

    // cube visual model
    auto mesh        = imstk::MeshIO::read(iMSTK_DATA_ROOT "/asianDragon/asianDragon.obj");
    auto SurfaceMesh = std::dynamic_pointer_cast<imstk::SurfaceMesh>(mesh);
    SurfaceMesh->scale(5., Geometry::TransformType::ApplyToData);
    auto renderModel = std::make_shared<VisualModel>(cubeGeom);
    auto mat         = std::make_shared<RenderMaterial>();
    mat->setDisplayMode(RenderMaterial::SURFACE);
    mat->setLineWidth(2.);
    mat->setColor(Color::Orange);
    renderModel->setRenderMaterial(mat);
    cubeObj->addVisualModel(renderModel);

    auto rigidMap = std::make_shared<IsometricMap>();
    rigidMap->setMaster(cubeGeom);
    rigidMap->setSlave(SurfaceMesh);

    // cube dynamic model
    auto rigidModel = std::make_shared<RigidBodyModel>();
    auto rigidProp  = std::make_shared<RigidBodyPropertyDesc>();

    rigidModel->configure(cubeGeom, rigidProp, RigidBodyType::Dynamic);
    cubeObj->setDynamicalModel(rigidModel);

    cubeObj->setPhysicsToVisualMap(rigidMap);

    // add cube to scene
    scene->addSceneObject(cubeObj);
    return cubeObj;
}

void
addPlaneRigidObject(std::shared_ptr<Scene> scene)
{
    // create plane object
    auto planeObj = std::make_shared<RigidObject>("Plane");

    auto planeGeom = std::make_shared<Plane>();
    planeGeom->setWidth(400.);

    // visual model
    auto renderModel2 = std::make_shared<VisualModel>(planeGeom);
    renderModel2->setRenderMaterial(std::make_shared<RenderMaterial>());
    planeObj->addVisualModel(renderModel2);

    // dynamic model
    auto rigidModel2 = std::make_shared<RigidBodyModel>();
    auto rigidProp2  = std::make_shared<RigidBodyPropertyDesc>();
    rigidModel2->configure(planeGeom, rigidProp2, RigidBodyType::Static);
    planeObj->setDynamicalModel(rigidModel2);

    scene->addSceneObject(planeObj);
}

void
addSphereRigidObject(std::shared_ptr<Scene> scene, Vec3d t = Vec3d(0., 0., 0.))
{
    // create cube object
    auto sphereObj = std::make_shared<RigidObject>("Sphere");

    // Create Cube object
    auto sphereGeom = std::make_shared<Sphere>();
    sphereGeom->setRadius(10.);
    sphereGeom->translate(t);

    // cube visual model
    auto renderModel = std::make_shared<VisualModel>(sphereGeom);
    renderModel->setRenderMaterial(std::make_shared<RenderMaterial>());
    sphereObj->addVisualModel(renderModel);

    // cube dynamic model
    auto rigidModel3 = std::make_shared<RigidBodyModel>();
    auto rigidProp   = std::make_shared<RigidBodyPropertyDesc>();
    rigidModel3->configure(sphereGeom, rigidProp, RigidBodyType::Dynamic);
    sphereObj->setDynamicalModel(rigidModel3);

    // add cube to scene
    scene->addSceneObject(sphereObj);
}

int
main()
{
    //simManager and Scene
    auto simManager = std::make_shared<SimulationManager>();
    auto scene      = simManager->createNewScene("Rigid Body Dynamics");

    auto cubeObj = addCubeRigidObject(std::string("cube"), scene, Vec3d(0., 0., 0.));

    addPlaneRigidObject(scene);
    //addSphereRigidObject(scene, Vec3d(0., 200, 0.));
    auto rigidObj = addMeshRigidObject(std::string("dragon"), scene, Vec3d(0., 30., 0.));

    //-------------------------------------------------------------

    // Device clients
    auto client = std::make_shared<HDAPIDeviceClient>(phantomOmni1Name);

    // Device Server
    auto server = std::make_shared<HDAPIDeviceServer>();
    server->addDeviceClient(client);
    simManager->addModule(server);

    // Create a virtual coupling object
    auto visualGeom = std::make_shared<Sphere>();
    visualGeom->setRadius(5.);
    auto obj         = std::make_shared<VisualObject>("VirtualCouplingObject");
    auto material    = std::make_shared<RenderMaterial>();
    auto visualModel = std::make_shared<VisualModel>(visualGeom);
    visualModel->setRenderMaterial(material);
    obj->addVisualModel(visualModel);
    scene->addSceneObject(obj);

    // Device tracker
    auto deviceTracker = std::make_shared<DeviceTracker>(client);
    auto objController = std::make_shared<imstk::SceneObjectController>(obj, deviceTracker);
    scene->addObjectController(objController);

    //-----------------------------------------------------------------

    auto rbModel = std::dynamic_pointer_cast<RigidBodyModel>(cubeObj->getDynamicalModel());

    if (!rbModel)
    {
        addCubeRigidObject(std::string("cube"), scene, Vec3d(0., 40., 0.));
        // throw error
    }
    auto prevCubePos = rbModel->getModelGeometry()->getTranslation();

    auto forceFunc =
        [&](Module* module)
        {
            auto devPos = deviceTracker->getPosition();
            auto devQ   = deviceTracker->getRotation();
            rbModel->getModelGeometry()->rotate(devQ);
            auto cubeGeo      = std::dynamic_pointer_cast<Cube>(cubeObj->getPhysicsGeometry());
            auto cubePos      = rbModel->getModelGeometry()->getTranslation();
            auto cubeVelocity = (cubePos - prevCubePos) / 2;
            auto damp         = -1000000 * cubeVelocity;
            auto force        = -1000 * (cubePos - devPos) + damp;
            rbModel->addForce(force, Vec3d(0., 0., 0.));
            prevCubePos = cubePos;
        };
    simManager->getSceneManager(scene)->setPreUpdateCallback(forceFunc);

    //-------------------------------------------------------------------

    // Set Camera configuration
    auto cam = scene->getCamera();
    cam->setPosition(Vec3d(300, 300, 300));

    // Light
    auto light = std::make_shared<DirectionalLight>("light");
    light->setIntensity(1);
    scene->addLight(light);

    // Run
    simManager->setActiveScene(scene);
    simManager->startSimulation(SimulationStatus::PAUSED);

    return 0;
}