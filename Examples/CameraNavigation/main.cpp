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
#include "imstkSphere.h"
#include "imstkCube.h"

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

using namespace imstk;

#define ADD_TOOL_CONTROLLER

//-------------------------------------------------
// Simulation presets
//-------------------------------------------------

// Select the scenario
const unsigned int scanarioNumber = 1; //possible values: 1, 2, 3, 4

// Camera settings
const double cameraAngulation = PI_4; //possible values: 0, 30, 45 deg
const double cameraViewAngle = 80.0; //possible values: 80 deg
const double cameraZoomFactor = 1.0; //possible values: TBD

//-------------------------------------------------

//-------------------------------------------------
// Simulation constants
//-------------------------------------------------

// Device names
const std::string device1Name("PHANToM 2");// Device 1 name
const std::string device2Name("PHANToM 1");// Device 2 name

const std::string pointerFileName("Resources/pencil.obj");// Pointer file name
const std::string patternTextureFileName("Resources/viewfinder.png");// Target texture file name

// Simulation constants
const int overlaySize = 300;

const double CameraBoundingBoxSize = 500.0;
const double cameraControllerScaling = 0.2;

//-------------------------------------------------

// Hold on the target for a certain amount of time
void createScenario1()
{
    std::cout << "Creating Scenario 1" << std::endl;

    // SDK and Scene
    auto sdk = std::make_shared<imstk::SimulationManager>();
    auto scene = sdk->createNewScene("CameraNavigationSimulator");

    // Add plane scene object
    auto planeGeom = std::make_shared<Plane>();
    planeGeom->scale(15);
    //planeGeom->translate(0, -1, 0);
    auto planeObj = std::make_shared<CollidingObject>("Plane");
    planeObj->setVisualGeometry(planeGeom);
    scene->addSceneObject(planeObj);

    // Create targets
    createTargetsScenario1(scene);

    // Device clients 2
    auto client0 = std::make_shared<imstk::HDAPIDeviceClient>(device1Name);

    // Device Server
    auto server = std::make_shared<imstk::HDAPIDeviceServer>();
    server->addDeviceClient(client0);

    // Update Camera position
    auto cam = scene->getCamera();
    cam->setPosition(Vec3d(0, 0, 20));
    cam->setViewAngle(cameraViewAngle);
    cam->setZoomFactor(cameraZoomFactor);

    // Set camera controller
    cam->setupController(client0, cameraControllerScaling);
    cam->getController()->setCameraRotationOffset(Quatd(Eigen::AngleAxisd(cameraAngulation, Vec3d::UnitY())));
    //cam->getController()->setInversionFlags(imstk::CameraController::InvertFlag::rotY | imstk::CameraController::InvertFlag::rotZ);

    sdk->addModule(server);

    // Set the scene as current
    sdk->setCurrentScene("CameraNavigationSimulator");

    // Add a 2D overlay on the 3D scene
    add2DTextureOverlay(sdk->getViewer()->getCurrentRenderer()->getVtkRenderer(), patternTextureFileName.c_str(), overlaySize);

    // Reset the clipping planes
    sdk->getViewer()->getCurrentRenderer()->getVtkRenderer()->ResetCameraClippingRange(-CameraBoundingBoxSize,
        CameraBoundingBoxSize,
        -CameraBoundingBoxSize,
        CameraBoundingBoxSize,
        -CameraBoundingBoxSize,
        CameraBoundingBoxSize);

    // Enable screen capture
    sdk->getViewer()->enableScreenCapture();

    sdk->startSimulation(true);
}

// Match a pattern on the target
void createScenario2()
{
    std::cout << "Creating Scenario 2" << std::endl;

    auto sdk = std::make_shared<imstk::SimulationManager>();
    auto scene = sdk->createNewScene("CameraNavigationSimulator");

    // Add plane scene object
    auto planeGeom = std::make_shared<Plane>();
    planeGeom->scale(10);
    //planeGeom->translate(0, -1, 0);
    auto planeObj = std::make_shared<CollidingObject>("Plane");
    planeObj->setVisualGeometry(planeGeom);
    scene->addSceneObject(planeObj);

    // Create targets
    createTargetsScenario2(scene);

    // Device clients 2
    auto client0 = std::make_shared<imstk::HDAPIDeviceClient>(device1Name);

    // Device Server
    auto server = std::make_shared<imstk::HDAPIDeviceServer>();
    server->addDeviceClient(client0);

    // Update Camera position
    auto cam = scene->getCamera();
    cam->setPosition(Vec3d(0, 0, 20));
    cam->setViewAngle(cameraViewAngle);
    cam->setZoomFactor(cameraZoomFactor);

    // Set camera controller
    cam->setupController(client0, cameraControllerScaling);
    cam->getController()->setCameraRotationOffset(Quatd(Eigen::AngleAxisd(cameraAngulation, Vec3d::UnitY())));
    //cam->getController()->setInversionFlags(imstk::CameraController::InvertFlag::rotY | imstk::CameraController::InvertFlag::rotZ);

#ifdef ADD_TOOL_CONTROLLER
    // Device clients 1
    auto client1 = std::make_shared<imstk::HDAPIDeviceClient>(device2Name);

    server->addDeviceClient(client1);

    // Load meshes
    auto mesh = imstk::MeshReader::read(pointerFileName);
    auto visualMesh = imstk::MeshReader::read(pointerFileName);

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
    sdk->setCurrentScene("CameraNavigationSimulator");

    // Reset the clipping planes
    sdk->getViewer()->getCurrentRenderer()->getVtkRenderer()->ResetCameraClippingRange(-CameraBoundingBoxSize,
        CameraBoundingBoxSize,
        -CameraBoundingBoxSize,
        CameraBoundingBoxSize,
        -CameraBoundingBoxSize,
        CameraBoundingBoxSize);

    sdk->startSimulation(true);
}

// Trace a pattern on the target
void createScenario3()
{
    std::cout << "Creating Scenario 3" << std::endl;

    auto sdk = std::make_shared<imstk::SimulationManager>();
    auto scene = sdk->createNewScene("CameraNavigationSimulator");

    // Add plane scene object
    auto planeGeom = std::make_shared<Plane>();
    planeGeom->scale(10);
    //planeGeom->translate(0, -1, 0);
    auto planeObj = std::make_shared<CollidingObject>("Plane");
    planeObj->setVisualGeometry(planeGeom);
    scene->addSceneObject(planeObj);

    // Create targets
    createTargetsScenario3(scene);

    // Device clients 2
    auto client0 = std::make_shared<imstk::HDAPIDeviceClient>(device1Name);

    // Device Server
    auto server = std::make_shared<imstk::HDAPIDeviceServer>();
    server->addDeviceClient(client0);

    // Update Camera position
    auto cam = scene->getCamera();
    cam->setPosition(imstk::Vec3d(0, 0, 20));
    cam->setViewAngle(cameraViewAngle);
    cam->setZoomFactor(cameraZoomFactor);

    // Set camera controller
    cam->setupController(client0, cameraControllerScaling);
    cam->getController()->setCameraRotationOffset(Quatd(Eigen::AngleAxisd(cameraAngulation, Vec3d::UnitY())));
    //cam->getController()->setInversionFlags(imstk::CameraController::InvertFlag::rotY | imstk::CameraController::InvertFlag::rotZ);

#ifdef ADD_TOOL_CONTROLLER
    // Device clients 1
    auto client1 = std::make_shared<imstk::HDAPIDeviceClient>(device2Name);

    server->addDeviceClient(client1);

    // Load meshes
    auto mesh = imstk::MeshReader::read(pointerFileName);
    auto visualMesh = imstk::MeshReader::read(pointerFileName);

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
    sdk->setCurrentScene("CameraNavigationSimulator");

    // Reset the clipping planes
    sdk->getViewer()->getCurrentRenderer()->getVtkRenderer()->ResetCameraClippingRange(-CameraBoundingBoxSize,
        CameraBoundingBoxSize,
        -CameraBoundingBoxSize,
        CameraBoundingBoxSize,
        -CameraBoundingBoxSize,
        CameraBoundingBoxSize);

    sdk->startSimulation(true);
}

// Test spatial awareness
void createScenario4()
{
    std::cout << "Creating Scenario 4" << std::endl;

    auto sdk = std::make_shared<imstk::SimulationManager>();
    auto scene = sdk->createNewScene("CameraNavigationSimulator");

    // Set the scene as current
    sdk->setCurrentScene("CameraNavigationSimulator");

    // Reset the clipping planes
    sdk->getViewer()->getCurrentRenderer()->getVtkRenderer()->ResetCameraClippingRange(-CameraBoundingBoxSize,
        CameraBoundingBoxSize,
        -CameraBoundingBoxSize,
        CameraBoundingBoxSize,
        -CameraBoundingBoxSize,
        CameraBoundingBoxSize);

    sdk->startSimulation(true);
}

// Driver code
int main()
{
    std::cout << "--------------------------------------\n"
            << "Starting Camera Navigation Application\n"
            << "--------------------------------------\n" << std::endl;

    // create the selected scenario
    switch (scanarioNumber)
    {
    case 1:
        createScenario1();
        break;
    case 2:
        createScenario2();
        break;
    case 3:
        createScenario3();
        break;
    case 4:
        createScenario4();
        break;
    default:
        std::cout << "Error: select the scenes from 1-4" << std::endl;
        break;
    }

    return 0;
}