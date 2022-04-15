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
	pbdParams->m_uniformMassValue = 0.01;
	pbdParams->m_gravity    = Vec3d(0.0, 0.0, -0.01);
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

	tetMesh->scale(0.02, Geometry::TransformType::ApplyToData);

	tetMesh->rotate(Vec3d(0.0, 0.0, 1.0), -PI_2, Geometry::TransformType::ApplyToData);
	tetMesh->rotate(Vec3d(1.0, 0.0, 0.0), -PI_2/6.0, Geometry::TransformType::ApplyToData);


	// Setup the Model
	auto pbdModel = std::make_shared<PbdModel>();
	pbdModel->setModelGeometry(tetMesh);
	pbdModel->configure(pbdParams);

	// Setup the material
	auto material = std::make_shared<RenderMaterial>();
	material->setDisplayMode(RenderMaterial::DisplayMode::WireframeSurface);

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


static std::shared_ptr<SceneObject>
makeClampObj(std::string name)
{
	auto surfMesh =
		MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "/Surgical Instruments/Clamps/Gregory Suture Clamp/gregory_suture_clamp.obj");

	surfMesh->scale(0.2, Geometry::TransformType::ApplyToData);


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
	
	scene->getActiveCamera()->setPosition(0.0, 0.0, 0.15);
	scene->getActiveCamera()->setFocalPoint(0.0, 0.0, -0.02);
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
	scene->addSceneObject(needleObj);

	// Add needle constraining behaviour between the tissue & arc needle
	// auto needleInteraction = std::make_shared<NeedleInteraction>(tissueHole, needleObj);
	// auto CD = needleInteraction->getCollisionDetection();
	//CD->set

	
		
	// scene->addInteraction(needleInteraction);






	// Create clamps that follow the needle around
	std::shared_ptr<SceneObject> toolObj = makeClampObj("Clamps");
	scene->addSceneObject(toolObj);

	// Create ghost clamps to show real position of hand under virtual coupling
	std::shared_ptr<SceneObject> ghostToolObj = makeClampObj("GhostClamps");
	ghostToolObj->getVisualModel(0)->getRenderMaterial()->setColor(Color::Orange);
	scene->addSceneObject(ghostToolObj);


	// Create the suture pbd-based string


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

		
		// scene->getConfig()->writeTaskGraph = true;

		driver->start();
	}


	return 0;
}