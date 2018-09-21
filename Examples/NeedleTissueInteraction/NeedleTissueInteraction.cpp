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

#include "imstkTimer.h"
#include "imstkSimulationManager.h"
#include "imstkForceModelConfig.h"
#include "imstkDeformableObject.h"
#include "imstkBackwardEuler.h"
#include "imstkNonlinearSystem.h"
#include "imstkNewtonSolver.h"
#include "imstkGaussSeidel.h"
#include "imstkConjugateGradient.h"
#include "imstkTetrahedralMesh.h"
#include "imstkMeshIO.h"
#include "imstkLineMesh.h"
#include "imstkOneToOneMap.h"
#include "imstkHDAPIDeviceClient.h"
#include "imstkHDAPIDeviceServer.h"
#include "imstkDeviceTracker.h"
#include "imstkSceneObjectController.h"
#include "LineToPointSetCD.h"
#include "NeedleTissueCH.h"
#include "imstkAPIUtilities.h"

using namespace imstk;

// Global variables
std::shared_ptr<Scene> g_scene;
std::shared_ptr<SimulationManager> g_sdk;
#ifdef iMSTK_USE_OPENHAPTICS
    std::shared_ptr<imstk::HDAPIDeviceClient> g_client;
    std::shared_ptr<imstk::HDAPIDeviceServer> g_server;
#endif
std::shared_ptr<imstk::DeviceTracker> g_deviceTracker;

// Needle tissue interaction configuration options
namespace NTISimulationConfig
{
    const double needleLength = 80.;
    const Vec3d needleStartPoint(0., 0., 0.9*needleLength);
    const Vec3d needleEndPoint(0., 0., -0.1*needleLength);
    const Color needleColor = Color::LightGray;

    const std::string phantomOmniName("Phantom1");

    const std::string kidneyMeshFilename(iMSTK_DATA_ROOT "/kidney/kidney.veg");
    const std::string kidneyConfigFilename(iMSTK_DATA_ROOT "/kidney/kidney.config");
    const std::string kidneyBCFilename(iMSTK_DATA_ROOT "/kidney/kidney.bou");

    const Vec3d centeringTransform(70., 30., -70.);
    const double geoScalingFactor = 1.;
    const double solverTolerance = 1.0e-6;
    const double forceScalingFactor = 1.0e-1;

    const double timeStep = 0.04;

    const Vec3d bgColor1(0.3285, 0.3285, 0.6525);
    const Vec3d bgColor2(0.1152, 0.1152, 0.2289);

    const bool dispayFPS = false;
    const bool runSimWithoutRendering = false;
    const bool renderDebugInfo = false;
}

// Create a needle object that is controlled by an external device
std::shared_ptr<CollidingObject>
createNeedle()
{
    std::vector<LineMesh::LineArray> lines;
    StdVectorOfVec3d points;
    std::vector<Color> colors;

    points.push_back(NTISimulationConfig::needleStartPoint);
    points.push_back(NTISimulationConfig::needleEndPoint);
    
    lines.push_back(LineMesh::LineArray({ 0, 1 }));
    
    colors.push_back(NTISimulationConfig::needleColor);
    colors.push_back(NTISimulationConfig::needleColor);

    // Construct line mesh
    auto lineMesh = std::make_shared<LineMesh>();
    lineMesh->initialize(points, lines);
    lineMesh->setVertexColors(colors);

    auto lineMeshMaterial = std::make_shared<RenderMaterial>();
    lineMeshMaterial->setLineWidth(3);
    lineMesh->setRenderMaterial(lineMeshMaterial);

    auto lineObject = std::make_shared<CollidingObject>("needleMesh");
    lineObject->setVisualGeometry(lineMesh);
    lineObject->setCollidingGeometry(lineMesh);

    g_scene->addSceneObject(lineObject);

    // add user control to the needle
#ifdef iMSTK_USE_OPENHAPTICS
    // Device clients
    g_client = std::make_shared<imstk::HDAPIDeviceClient>(NTISimulationConfig::phantomOmniName);

    // Device Server
    g_server = std::make_shared<imstk::HDAPIDeviceServer>();
    g_server->addDeviceClient(g_client);
    g_sdk->addModule(g_server);

    // Device tracker
    g_deviceTracker = std::make_shared<imstk::DeviceTracker>(g_client);

    auto needleController = std::make_shared<imstk::SceneObjectController>(lineObject, g_deviceTracker);
    g_scene->addObjectController(needleController);

#endif
    return lineObject;
}

bool
loadBoundaryConditions(std::vector<int>& bcNodeList)
{
    std::string fileName(NTISimulationConfig::kidneyBCFilename);

    FILE *fp = fopen(fileName.data(), "rb");
    if (!fp)
    {
        return false;
    }

    int nodeNum;
    while (fscanf(fp, "%d ", &nodeNum) != EOF)
    {
        bcNodeList.emplace_back(nodeNum);
    }
    fclose(fp);
    std::sort(bcNodeList.begin(), bcNodeList.end());
    return true;
}

int main()
{
    // Create simulation manager and Scene
    g_sdk = std::make_shared<SimulationManager>(NTISimulationConfig::runSimWithoutRendering);
    g_scene = g_sdk->createNewScene("NeedleTissueInteraction");
    g_scene->getCamera()->setPosition(0, 2.0, 15.0);

    // Load a kidney mesh
    auto tetMesh = MeshIO::read(NTISimulationConfig::kidneyMeshFilename);
    if (!tetMesh)
    {
        LOG(WARNING) << "Could not read mesh from file: " << NTISimulationConfig::kidneyConfigFilename;
        return 1;
    }

    // Extract the surface mesh from the tetrahedral mesh
    auto surfMesh = std::make_shared<SurfaceMesh>();
    auto volTetMesh = std::dynamic_pointer_cast<TetrahedralMesh>(tetMesh);

    volTetMesh->scale(NTISimulationConfig::geoScalingFactor, Geometry::TransformType::ApplyToData);
    volTetMesh->translate(NTISimulationConfig::centeringTransform, Geometry::TransformType::ApplyToData);
    if (!volTetMesh)
    {
        LOG(WARNING) << "Dynamic pointer cast from PointSet to TetrahedralMesh failed!";
        return 1;
    }
    volTetMesh->extractSurfaceMesh(surfMesh, true);

    // Construct one to one nodal map based on the above meshes
    auto oneToOneNodalMap = std::make_shared<OneToOneMap>();
    oneToOneNodalMap->setMaster(tetMesh);
    oneToOneNodalMap->setSlave(surfMesh);

    // Compute the map
    oneToOneNodalMap->compute();

    // Configure dynamic model
    auto dynaModel = std::make_shared<FEMDeformableBodyModel>();
    dynaModel->configure(NTISimulationConfig::kidneyConfigFilename);
    //dynaModel->setTimeStepSizeType(TimeSteppingType::realTime);
    dynaModel->setModelGeometry(volTetMesh);
    auto timeIntegrator = std::make_shared<BackwardEuler>(NTISimulationConfig::timeStep);
    dynaModel->setTimeIntegrator(timeIntegrator);

    auto material = std::make_shared<RenderMaterial>();
    material->setDisplayMode(RenderMaterial::DisplayMode::WIREFRAME_SURFACE);    
    material->setPointSize(4.0);
    material->setLineWidth(2.0);
    surfMesh->setRenderMaterial(material);

    // Scene Object
    auto deformableObj = std::make_shared<DeformableObject>("Kidney");
    deformableObj->setVisualGeometry(surfMesh);
    deformableObj->setPhysicsGeometry(volTetMesh);
    deformableObj->setPhysicsToVisualMap(oneToOneNodalMap); //assign the computed map
    deformableObj->setDynamicalModel(dynaModel);
    g_scene->addSceneObject(deformableObj);    

    // create a nonlinear system
    auto nlSystem = std::make_shared<NonLinearSystem>(
        dynaModel->getFunction(),
        dynaModel->getFunctionGradient());

    // Add boundary conditions
    std::vector<LinearProjectionConstraint> projList;
    std::vector<int> nodeBCList;
    nodeBCList.reserve(400);
    loadBoundaryConditions(nodeBCList);
    for (auto& i : nodeBCList)
    {
        projList.push_back(LinearProjectionConstraint(i, true));
    }

    nlSystem->setUnknownVector(dynaModel->getUnknownVec());
    nlSystem->setUpdateFunction(dynaModel->getUpdateFunction());
    nlSystem->setUpdatePreviousStatesFunction(dynaModel->getUpdatePrevStateFunction());

    // create a linear solver
    auto linSolver = std::make_shared<ConjugateGradient>();
    linSolver->setTolerance(NTISimulationConfig::solverTolerance);
    linSolver->setLinearProjectors(&projList);

    // create a non-linear solver and add to the scene
    auto nlSolver = std::make_shared<NewtonSolver>();
    nlSolver->setLinearSolver(linSolver);
    nlSolver->setSystem(nlSystem);
    g_scene->addNonlinearSolver(nlSolver);

    auto needleObject = createNeedle();

    // Add collision detection and handling
    std::vector<LinearProjectionConstraint> needleProjList;
    linSolver->setDynamicLinearProjectors(&needleProjList);
    
    CollisionData colData;
    colData.NeedleColData.resize(0);
    colData.NeedleColData.reserve(400);

    if (g_deviceTracker)
    {
        // create collision detection
        auto CD = std::make_shared<LineToPointSetCD>(std::dynamic_pointer_cast<PointSet>(volTetMesh),
            g_deviceTracker,
            NTISimulationConfig::needleStartPoint,
            NTISimulationConfig::needleEndPoint,
            colData);

        // collision handling
        auto CHA = std::make_shared<NeedleTissueInteraction>(CollisionHandling::Side::A,
            colData,
            &needleProjList,
            needleObject,
            deformableObj);

        CHA->setScalingFactor(NTISimulationConfig::forceScalingFactor);

        // Add the interaction pair to the scene
        g_scene->getCollisionGraph()->addInteractionPair(std::dynamic_pointer_cast<CollidingObject>(deformableObj),
            needleObject,
            CD,
            nullptr,
            CHA);
    }

    // print frame rate
    if (NTISimulationConfig::dispayFPS)
    {
        auto ups = std::make_shared<UPSCounter>();
        apiutils::printUPS(g_sdk->getSceneManager(g_scene), ups);
    }

    // render debug info
    if (NTISimulationConfig::renderDebugInfo)
    {
        auto constrainedNodesDisplay = std::make_shared<PointSet>();
        StdVectorOfVec3d dbgPointList(400, Vec3d(0., 0., 0.));
        constrainedNodesDisplay->initialize(dbgPointList);

        auto dbgMaterial = std::make_shared<RenderMaterial>();
        dbgMaterial->setDisplayMode(RenderMaterial::DisplayMode::POINTS);
        dbgMaterial->setPointSize(6.0);
        dbgMaterial->setColor(imstk::Color::Pink);
        constrainedNodesDisplay->setRenderMaterial(dbgMaterial);

        auto constrainedNodesObj = std::make_shared<VisualObject>("debugDisplayObj");
        constrainedNodesObj->setVisualGeometry(constrainedNodesDisplay);
        g_scene->addSceneObject(constrainedNodesObj);

        auto udpdateConstrNodes = [&](Module* module)
        {
            if (colData.NeedleColData.size() != 0)
            {
                for (auto& c : colData.NeedleColData)
                {
                    constrainedNodesDisplay->setVertexPosition(c.nodeId, volTetMesh->getVertexPosition(c.nodeId));
                }

                for (int i = colData.NeedleColData.size(); i < dbgPointList.size(); ++i)
                {
                    constrainedNodesDisplay->setVertexPosition(i, Vec3d(0., 0., 0.));
                }
            }
        };
        g_sdk->getSceneManager(g_scene)->setPostUpdateCallback(udpdateConstrNodes);
    }

    // Set up Light
    auto light = std::make_shared<DirectionalLight>("light1");    
    light->setFocalPoint(Vec3d(5, -8, -5));
    light->setIntensity(2);
    g_scene->addLight(light);

    // Set up Camera
    auto camera = g_scene->getCamera();
    camera->setPosition(imstk::Vec3d(0., 30., 100.));
    camera->setFocalPoint(imstk::Vec3d(0., 0., 0.));

    // Run the simulation
    g_sdk->setActiveScene(g_scene);
    if (!NTISimulationConfig::runSimWithoutRendering)
    {
        g_sdk->getViewer()->setBackgroundColors(NTISimulationConfig::bgColor1, NTISimulationConfig::bgColor2, true);
    }
    g_sdk->startSimulation(SimulationStatus::RUNNING); // start in running mode

    // Perform an infinite loop if there is no rendering enabled
    if (NTISimulationConfig::runSimWithoutRendering)
    {
        LOG(INFO) << "simulation is starting. PRESS any key to exit";
        while (g_sdk->getStatus() == SimulationStatus::RUNNING && !getchar()) {}
        g_sdk->endSimulation();
    }

    return 0;
}