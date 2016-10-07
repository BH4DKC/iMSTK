#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <iomanip>

#include "imstkMath.h"
#include "imstkTimer.h"
#include "imstkSimulationManager.h"
#include "imstkViewer.h"
#include "CameraNavigationUtils.h"

// Objects
#include "imstkSceneObject.h"
#include "imstkVirtualCouplingObject.h"
#include "imstkLight.h"
#include "imstkCamera.h"

#include "imstkIsometricMap.h"

// Devices
#include "imstkHDAPIDeviceClient.h"
#include "imstkHDAPIDeviceServer.h"
#include "imstkVRPNDeviceClient.h"
#include "imstkVRPNDeviceServer.h"
#include "imstkCameraController.h"

// logger
#include "g3log/g3log.hpp"
#include "imstkUtils.h"

#define ADD_TOOL_CONTROLLER

using namespace imstk;

// Select the scenario
const unsigned int scanarioNumber = 1;

// Camera settings
const double cameraAngulation = 0.0;// PI / 6.0;
const double cameraViewAngle = 80.0;
const double cameraZoomFactor = 1.0;

// Hold on the target for a certain amount of time
void createScenario1(std::shared_ptr<imstk::SimulationManager>& sdk, std::shared_ptr<imstk::Scene>& scene)
{
    // Add plane scene object
    auto planeGeom = std::make_shared<Plane>();
    planeGeom->scale(10);
    planeGeom->translate(0, -1, 0);
    auto planeObj = std::make_shared<CollidingObject>("Plane");
    planeObj->setVisualGeometry(planeGeom);
    scene->addSceneObject(planeObj);

    // Create targets
    createTargets(scene);

    // Device clients 2
    auto client0 = std::make_shared<imstk::HDAPIDeviceClient>("PHANToM 1");

    // Device Server
    auto server = std::make_shared<imstk::HDAPIDeviceServer>();
    server->addDeviceClient(client0);

    // Update Camera position
    auto cam = scene->getCamera();
    cam->setPosition(imstk::Vec3d(0, 0, 20));
    cam->setViewAngle(cameraViewAngle);
    cam->setZoomFactor(cameraZoomFactor);

    // Set camera controller
    cam->setupController(client0, 0.4);
    cam->getController()->setRotationOffset(Quatd(Eigen::AngleAxisd(cameraAngulation, Vec3d::UnitY())));
    cam->getController()->setInversionFlags(imstk::CameraController::InvertFlag::rotY | imstk::CameraController::InvertFlag::rotZ);

#ifdef ADD_TOOL_CONTROLLER
    // Device clients 1
    auto client1 = std::make_shared<imstk::HDAPIDeviceClient>("PHANToM 2");

    server->addDeviceClient(client1);

    // Load meshes
    std::string path2obj = "Resources/handle2.obj";
    auto mesh = imstk::MeshReader::read(path2obj);
    auto visualMesh = imstk::MeshReader::read(path2obj);

    // construct map
    auto C2VHandle = std::make_shared<imstk::IsometricMap>();
    C2VHandle->setMaster(mesh);
    C2VHandle->setSlave(visualMesh);
    C2VHandle->compute();

    // create virtual tool
    auto handle = std::make_shared<imstk::VirtualCouplingObject>("tool", client1, 0.5);
    handle->setCollidingGeometry(mesh);
    handle->setVisualGeometry(mesh);
    handle->setCollidingToVisualMap(C2VHandle);

    // add virtual tool to the scene
    scene->addSceneObject(handle);
#endif

    sdk->addModule(server);

    // Set the scene as current
    sdk->setCurrentScene("Camera Navigation simulator");
}

// Match a pattern on the target
void createScenario2(std::shared_ptr<imstk::SimulationManager>& sdk, std::shared_ptr<imstk::Scene>& scene)
{
    // Add a sample scene object
    /*auto dragonMesh = imstk::MeshReader::read("Resources/asianDragon/asianDragon.obj");
    auto dragonObject = std::make_shared<imstk::VisualObject>("dragonObject");
    dragonObject->setVisualGeometry(dragonMesh);
    scene->addSceneObject(dragonObject);*/

    // Add plane scene object
    auto planeGeom = std::make_shared<Plane>();
    planeGeom->scale(10);
    planeGeom->translate(0, -1, 0);
    auto planeObj = std::make_shared<CollidingObject>("Plane");
    planeObj->setVisualGeometry(planeGeom);
    scene->addSceneObject(planeObj);

    // Create targets
    createTargets(scene);

    // Set the scene as current
    sdk->setCurrentScene("Camera Navigation simulator");

    // Add a 2D overlay on the 3D scene
    add2DTextureOverlay(sdk->getViewer()->getCurrentRenderer()->getVtkRenderer(), "Resources/viewfinder.png");
}

// Trace a pattern on the target
void createScenario3(std::shared_ptr<imstk::SimulationManager>& sdk, std::shared_ptr<imstk::Scene>& scene)
{

}

// Test spatial awareness
void createScenario4(std::shared_ptr<imstk::SimulationManager>& sdk, std::shared_ptr<imstk::Scene>& scene)
{

}

// Driver code
int main()
{
    std::cout << "****************\n"
        << "Starting Camera Navigation Application"
        << "****************\n";

    // create sdk and scene
    auto sdk = std::make_shared<SimulationManager>();
    auto scene = sdk->createNewScene("Camera Navigation simulator");

    switch (scanarioNumber)
    {
    case 1:
        createScenario1(sdk, scene);
        break;
    case 2:
        createScenario2(sdk, scene);
        break;
    case 3:
        createScenario3(sdk, scene);
        break;
    case 4:
        createScenario4(sdk, scene);
        break;
    default:
        std::cout << "Error: select the scenes from 1-4" << std::endl;
        break;
    }

    // Run
    sdk->startSimulation(true);
}