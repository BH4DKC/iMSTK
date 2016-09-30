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
#include "imstkVRPNDeviceClient.h"
#include "imstkVRPNDeviceServer.h"
#include "imstkCameraController.h"

// logger
#include "g3log/g3log.hpp"
#include "imstkUtils.h"

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

//#define ADD_TOOL_CONTROLLER

using namespace imstk;

// Texture coordinates
const imstk::Vec2d texCenter(750.0, 750.0);
const imstk::Vec2d texCircleTop(750.0, 250.0);
const imstk::Vec2d texCircleBottom(750.0, 1250.0);
const imstk::Vec2d texTopLeftCorner(252.0, 665.0);
const imstk::Vec2d texTopRightCorner(1254.0, 665.0);
const imstk::Vec2d texBottomRightCorner(1254.0, 832.0);
const imstk::Vec2d texBottomLeftCorner(252.0, 832.0);

template <typename T>
struct targetPoints
{
	T center, top, bottom;
	T corners[4];// in clockwise from top left
};

typedef targetPoints<imstk::Vec2d> screenSpacePoints;
typedef targetPoints<imstk::Vec3d> targetPointsInWorld;
typedef targetPoints<imstk::Vec3d> screenSpacePtsWithDepth;

targetPointsInWorld targetWorldPoints[6];
const double cameraAngulation = 0.0;// PI / 6.0;
const double cameraViewAngle = 80.0;
const double cameraZoomFactor = 1.0;

///
/// \brief Create the target blocks
///
void createTargets(std::shared_ptr<imstk::Scene>& scene)
{
	// some constants
	const float X = 8;
	const float Y = 6;
	const float Z = 6;
	const float pY = 0.25;
	const float pZ = 0.25;
	const double radius = 3.0;
	const double scaling = 0.15;
	const double planeWidth = 10;

	//imstk::Color meshColor(0.25, 0.25, 0.25, 1.0);

	//auto blockRenderDetail = std::make_shared<imstk::RenderDetail>();//IMSTK_RENDER_NORMALS
	//blockRenderDetail->setAmbientColor(meshColor);
	//blockRenderDetail->setDiffuseColor(meshColor);
	//blockRenderDetail->setSpecularColor(meshColor);
	//blockRenderDetail->setShininess(100.0);

	for (int i = 0; i < 6; i++)
	{
		// transformations
		Eigen::UniformScaling<double> s(scaling);
		Eigen::Translation3d t1(0, 0, -radius);
		Eigen::Translation3d t2(0, 0, -radius + 0.01);
		Eigen::Quaterniond q(cos(i*22.0 / 42), 0, sin(i*22.0 / 42), 0);
		q.normalize();

		// BLOCKS
		// surface mesh
		std::vector<imstk::Vec3d> blockPts = { imstk::Vec3d(X / 2, 0, -Z / 2), imstk::Vec3d(X / 2, 0, Z / 2),
			imstk::Vec3d(-X / 2, 0, Z / 2), imstk::Vec3d(-X / 2, 0, -Z / 2),
			imstk::Vec3d(-X / 2, Y, -Z / 2), imstk::Vec3d(X / 2, Y, -Z / 2),
			imstk::Vec3d(-X / 2, Y, Z*(pZ - 0.5)), imstk::Vec3d(X / 2, Y, Z*(pZ - 0.5)),
			imstk::Vec3d(-X / 2, Y*pY, Z / 2), imstk::Vec3d(X / 2, Y*pY, Z / 2) };

		std::vector<std::array<size_t, 3>> blockTriangles = { { { 0, 1, 2 } }, { { 0, 2, 3 } },
		{ { 0, 3, 4 } }, { { 5, 0, 4 } },
		{ { 5, 4, 6 } }, { { 7, 5, 6 } },
		{ { 6, 8, 9 } }, { { 6, 9, 7 } },
		{ { 2, 1, 9 } }, { { 8, 2, 9 } },
		{ { 3, 6, 4 } }, { { 3, 8, 6 } },
		{ { 3, 2, 8 } }, { { 5, 7, 0 } },
		{ { 7, 9, 0 } }, { { 9, 1, 0 } } };

		// scale, translate, rotate (fix in architecture)
		/*for (int j = 0; j < 10; j++)
		{
			blockPts[j] *= scaling;
			blockPts[j] += imstk::Vec3d(0, 0, -radius);
			blockPts[j] = q*blockPts[j];
		}*/

		auto blockMesh = std::make_shared<imstk::SurfaceMesh>();
		blockMesh->initialize(blockPts, blockTriangles, true);
		blockMesh->scale(scaling);
		blockMesh->translate(Vec3d(0, 0, -radius));
		blockMesh->rotate(q);
		
		// add object to the scene
		auto blockObject = std::make_shared<imstk::VisualObject>("Target " + std::to_string(i));
		blockObject->setVisualGeometry(blockMesh);
		scene->addSceneObject(blockObject);

		//------------------------------------------------------


		std::string planeName("Plane " + std::to_string(i));
		std::string textureName("Resources/target.png");
		createPlaneTargetWithTexture(scene, 1.0, Eigen::Translation3d(2, 0, 0), q, textureName, planeName);

		// Read surface mesh
		//auto objMesh = imstk::MeshReader::read("Resources/plane.obj");
		//auto surfaceMesh = std::dynamic_pointer_cast<imstk::SurfaceMesh>(objMesh);
		//surfaceMesh->addTexture("Resources/target.png");

		//// Create object and add to scene
		//auto object = std::make_shared<imstk::VisualObject>("meshObject");
		//object->setVisualGeometry(surfaceMesh); // change to any mesh created above
		//scene->addSceneObject(object);

		//// TARGETS
		//// Some constants
		//const double L_3d = blockPts[7].x() - blockPts[6].x();
		//const double H_3d = (blockPts[8] - blockPts[6]).norm();
		//const double delta = 0.5*(L_3d - H_3d);
		//const double yL = blockPts[6].y() - blockPts[8].y();
		//const double zL = blockPts[8].z() - blockPts[6].z();

		//imstk::Vec3d pointA(blockPts[6].x() + delta, blockPts[6].y(), blockPts[6].z());

		//// Set the target points
		//double y6 = blockPts[6].y();
		//double z6 = blockPts[6].z();
		//targetWorldPoints[i].center = imstk::Vec3d(0.5*(blockPts[6] + blockPts[9]));
		//targetWorldPoints[i].top = imstk::Vec3d(pointA.x() + H_3d / 2, y6 - yL / 6, z6 + zL / 6);
		//targetWorldPoints[i].bottom = imstk::Vec3d(pointA.x() + H_3d / 2, y6 - 5 * yL / 6, z6 + 5 * zL / 6);
		//targetWorldPoints[i].corners[0] = imstk::Vec3d(pointA.x() + 252.0*H_3d / 1500, y6 - 665.0*yL / 1500, z6 + 665.0*zL / 1500);
		//targetWorldPoints[i].corners[1] = imstk::Vec3d(pointA.x() + 1254.0*H_3d / 1500, y6 - 665.0*yL / 1500, z6 + 665.0*zL / 1500);
		//targetWorldPoints[i].corners[2] = imstk::Vec3d(pointA.x() + 1254.0*H_3d / 1500, y6 - 832.0*yL / 1500, z6 + 1254.0*zL / 1500);
		//targetWorldPoints[i].corners[3] = imstk::Vec3d(pointA.x() + 252.0*H_3d / 1500, y6 - 832.0*yL / 1500, z6 + 1254.0*zL / 1500);

		//auto targetRenderDetail = std::make_shared<imstk::RenderDetail>(IMSTK_RENDER_TEXTURE);
		//targetRenderDetail->addTexture("target", targetFileName, "", "");
		//// surface mesh
		//imstk::Vec3d topLeftEdge(-X / 2, Y, Z*(pZ - 0.5));
		//imstk::Vec3d topRightEdge(X / 2, Y, Z*(pZ - 0.5));
		//imstk::Vec3d bottomLeftEdge(-X / 2, Y*pY, Z / 2);
		//imstk::Vec3d bottomRightEdge(X / 2, Y*pY, Z / 2);
		//std::vector<imstk::Vec3d> points = { topLeftEdge, topRightEdge, bottomLeftEdge, bottomRightEdge };
		//std::array<size_t, 3> tri1 = { { 0, 1, 2 } };
		//std::array<size_t, 3> tri2 = { { 1, 2, 3 } };
		//std::vector<std::array<size_t, 3>> triArray = { tri1, tri2 };
		//auto surfaceMesh = std::make_shared<imstk::SurfaceMesh>();
		//surfaceMesh->initialize(points, triArray);

		//// Texture Coordinates
		///*double height = (bottomLeftEdge - topLeftEdge).norm();
		//double width = (bottomLeftEdge - bottomRightEdge).norm();
		//double dist = std::min(height, width);
		//double dx = 0.0, dy = 0.0;
		//if (dist == height)
		//{
		//	double x_a = bottomLeftEdge[0];
		//	double x_b = bottomRightEdge[0];
		//	dx = std::abs((x_b - x_a - dist) / (2 * dist));
		//}
		//else
		//{
		//	double y_a = bottomLeftEdge[1];
		//	double y_b = topLeftEdge[1];
		//	dy = std::abs((y_b - y_a - dist) / (2 * dist));
		//}
		//surfaceMesh->addTextureCoordinate(1.0 + dx, -dy);
		//surfaceMesh->addTextureCoordinate(-dx, -dy);
		//surfaceMesh->addTextureCoordinate(1.0 + dx, 1.0 + dy);
		//surfaceMesh->addTextureCoordinate(-dx, 1.0 + dy);

		//surfaceMesh->setUseOBJTexture(true);*/

		//// model
		//auto targetModel = std::make_shared<imstk::MeshCollisionModel>();
		//targetModel->setMesh(surfaceMesh);
		//targetModel->setRenderDetail(targetRenderDetail);
		//targetModel->getMesh()->scale(s);
		//targetModel->getMesh()->translate(t2);
		//targetModel->getMesh()->rotate(q);

		//// Scale, Translate and rotate the points
		//imstk::Vec3d tra(0, 0, -radius + 0.01);
		//targetWorldPoints[i].center = q*(s*targetWorldPoints[i].center + tra);
		//targetWorldPoints[i].top = q*(s*targetWorldPoints[i].top + tra);
		//targetWorldPoints[i].bottom = q*(s*targetWorldPoints[i].bottom + tra);
		//for (int j = 0; j < 4; j++)
		//{
		//	targetWorldPoints[i].corners[j] = q*(s*targetWorldPoints[i].corners[j] + tra);
		//}

		//// object
		//auto targetObject = std::make_shared<imstk::VisualObject>();
		//targetObject->setVisualGeometry(targetModel);
		//scene->addSceneObject(targetObject);
	}
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

#ifdef ADD_TOOL_CONTROLLER
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
#endif

	// CAMERA CONTROLLER

	// Device clients 2
	auto client1 = std::make_shared<imstk::HDAPIDeviceClient>("PHANToM 2");
	sdk->addDeviceClient(client1);

	// Update Camera position
	auto cam = scene->getCamera();
	cam->setPosition(imstk::Vec3d(0, 0, 20));
	cam->setViewAngle(cameraViewAngle);
	cam->setZoomFactor(cameraZoomFactor);

	// Set camera controller
	cam->setupController(client1, 0.4);	
	cam->getController()->setRotationOffset(Quatd(Eigen::AngleAxisd(cameraAngulation, Vec3d::UnitY())));
	cam->getController()->setInversionFlags(imstk::CameraController::InvertFlag::rotY | imstk::CameraController::InvertFlag::rotZ);

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

	// Run
	sdk->startSimulation(true);
}