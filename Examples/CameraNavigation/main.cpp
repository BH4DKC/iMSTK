#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <iomanip>

#include "imstkMath.h"
#include "imstkTimer.h"
#include "imstkSimulationManager.h"
#include "imstkViewer.h"

// Objects
#include "imstkForceModelConfig.h"
#include "imstkDeformableBodyModel.h"
#include "imstkDeformableObject.h"
#include "imstkSceneObject.h"
#include "imstkVirtualCouplingObject.h"
#include "imstkLight.h"
#include "imstkCamera.h"

// Time Integrators
#include "imstkBackwardEuler.h"

// Solvers
#include "imstkNonlinearSystem.h"
#include "imstkNewtonMethod.h"
#include "imstkConjugateGradient.h"

// Geometry
#include "imstkPlane.h"
#include "imstkSphere.h"
#include "imstkCube.h"
#include "imstkTetrahedralMesh.h"
#include "imstkSurfaceMesh.h"
#include "imstkMeshReader.h"
#include "imstkLineMesh.h"

// Maps
#include "imstkTetraTriangleMap.h"
#include "imstkIsometricMap.h"
#include "imstkOneToOneMap.h"

// Devices
#include "imstkHDAPIDeviceClient.h"
#include "imstkVRPNDeviceClient.h"
#include "imstkVRPNDeviceServer.h"
#include "imstkCameraController.h"

// Collisions
#include "imstkInteractionPair.h"

// logger
#include "g3log/g3log.hpp"
#include "imstkUtils.h"

#include "imstkVirtualCouplingPBDObject.h"
#include "imstkPbdObject.h"

// testVTKTexture
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <string>
#include <vtkJPEGReader.h>

// Overlay
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <vtkImageResize.h>
#include <vtkImageTranslateExtent.h>
#include <vtkImageMapper.h>
#include <vtkActor2D.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkImageResize.h>
#include <vtkImageTranslateExtent.h>

using namespace imstk;

const int overlaySize = 400;

///
///	 \brief Add a 2D overlay of target markers on a 3D scene
///
void add2DOverlay(vtkSmartPointer<vtkRenderer> rendererVtk, const char* fileName)
{
	// Read the image
	vtkNew<vtkPNGReader> reader;
	reader->SetFileName(fileName);
	reader->Update();

	int dim[3] = { overlaySize, overlaySize, 1 };

	// Resize image
	vtkNew<vtkImageResize> resize;
	resize->SetInputConnection(reader->GetOutputPort());
	resize->SetOutputDimensions(dim);

	// Translate image extent (origin to its center)
	vtkNew<vtkImageTranslateExtent> translateExtent;
	translateExtent->SetInputConnection(resize->GetOutputPort());
	translateExtent->SetTranslation(-dim[0] / 2, -dim[1] / 2, 0);

	// Mapper
	vtkNew<vtkImageMapper> imageMapper;
	imageMapper->SetInputConnection(translateExtent->GetOutputPort());
	imageMapper->SetColorWindow(255);
	imageMapper->SetColorLevel(127);

	// Actor
	vtkNew<vtkActor2D> imageActor;
	imageActor->SetMapper(imageMapper.GetPointer());
	imageActor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedDisplay();
	imageActor->SetPosition(0.5, 0.5);

	// Renderer
	rendererVtk->AddActor2D(imageActor.GetPointer());
}

int main()
{
	std::cout << "****************\n"
			  << "Starting Camera Navigation Application\n"
			  << "****************\n";

	// create sdk
	auto sdk = std::make_shared<SimulationManager>();
	auto scene = sdk->createNewScene("Camera Navigation simulator");

	// TOOL CONTROLLER

	// Device clients 1
	auto client0 = std::make_shared<imstk::HDAPIDeviceClient>("PHANToM 1");
	sdk->addDeviceClient(client0);

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
	auto handle = std::make_shared<imstk::VirtualCouplingObject>("tool", client0, 0.5);
	handle->setCollidingGeometry(mesh);
	handle->setVisualGeometry(mesh);
	handle->setCollidingToVisualMap(C2VHandle);

	// add virtual tool to the scene
	scene->addSceneObject(handle);

	// CAMERA CONTROLLER

	// Device clients 2
	auto client1 = std::make_shared<imstk::HDAPIDeviceClient>("PHANToM 2");
	sdk->addDeviceClient(client1);

	// Update Camera position
	auto cam = scene->getCamera();
	cam->setPosition(imstk::Vec3d(0, 0, 20));

	// Set camera controller
	cam->setupController(client1, 0.4);
	cam->getController()->setInversionFlags(imstk::CameraController::InvertFlag::rotY | imstk::CameraController::InvertFlag::rotZ);

	// Add a sample scene object
	auto dragonMesh = imstk::MeshReader::read("Resources/asianDragon/asianDragon.obj");
	auto dragonObject = std::make_shared<imstk::VisualObject>("dragonObject");
	dragonObject->setVisualGeometry(dragonMesh);
	scene->addSceneObject(dragonObject);

	// Add plane scene object
	auto planeGeom = std::make_shared<Plane>();
	planeGeom->scale(40);
	planeGeom->translate(0, -6, 0);
	auto planeObj = std::make_shared<CollidingObject>("Plane");
	planeObj->setVisualGeometry(planeGeom);
	scene->addSceneObject(planeObj);

	// Set the scene as current
	sdk->setCurrentScene("Camera Navigation simulator");

	// Add a 2D overlay on the 3D scene
	add2DOverlay(sdk->getViewer()->getCurrentRenderer()->getVtkRenderer(), "Resources/viewfinder.png");

	// Run
	sdk->startSimulation(true);
}