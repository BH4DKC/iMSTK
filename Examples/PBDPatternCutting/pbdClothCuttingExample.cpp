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
#include "imstkPbdModel.h"
#include "imstkPbdObject.h"
#include "imstkPbdSolver.h"
#include "imstkAPIUtilities.h"
#include "imstkSurfaceCuttingManager.h"
#include "imstkHDAPIDeviceServer.h"
#include "imstkHDAPIDeviceClient.h"
#include "imstkSceneObjectController.h"
#include "imstkMeshToMeshBruteForceCD.h"
#include "imstkPointSetToPlaneCD.h"
#include "imstkPBDCollisionHandling.h"
#include "imstkLaparoscopicToolController.h"
#include "imstkCylinder.h"
#include "imstkDebugRenderGeometry.h"
#include "imstkVTKViewer.h"
#include "imstkVTKTextStatusManager.h"
using namespace imstk;
const std::string phantomOmni1Name = "Phantom1";
const std::string phantomOmni2Name = "Phantom2";

//Functions to add DebugRender Geometries 
std::shared_ptr<DebugRenderGeometry>
addTrianglesDebugRendering(const std::shared_ptr<Scene>& scene, Color color = Color::Orange)
{
    auto debugTriangles = std::make_shared<DebugRenderTriangles>("Debug Triangles");
    auto material = std::make_shared<RenderMaterial>();
    material->setBackFaceCulling(false);
    material->setDebugColor(Color::Red);
    material->setColor(color);

    material->setDisplayMode(RenderMaterial::DisplayMode::WIREFRAME_SURFACE);
    debugTriangles->setRenderMaterial(material);
    scene->addDebugGeometry(debugTriangles);

    return std::dynamic_pointer_cast<DebugRenderGeometry>(debugTriangles);
}

std::shared_ptr<DebugRenderGeometry>
addLinesDebugRendering(const std::shared_ptr<Scene>& scene, std::string name = std::string("Debug Lines"), Color color = Color::Green, float lineWidth = 2.0)
{
    auto debugLines = std::make_shared<DebugRenderLines>(name);
    auto material = std::make_shared<RenderMaterial>();
    material->setBackFaceCulling(false);
    material->setDebugColor(color);
    material->setLineWidth(lineWidth);
    debugLines->setRenderMaterial(material);
    scene->addDebugGeometry(debugLines);
    return std::dynamic_pointer_cast<DebugRenderGeometry>(debugLines);
}

std::shared_ptr<DebugRenderGeometry>
addPointsDebugRendering(const std::shared_ptr<Scene>& scene)
{
    auto debugPoints = std::make_shared<DebugRenderPoints>("Debug Points");
    auto material = std::make_shared<RenderMaterial>();
    material->setBackFaceCulling(false);
    material->setColor(Color::Green);
    material->setPointSize(0.1); // Issue : setPointSize doesn't work!
    debugPoints->setRenderMaterial(material);
    scene->addDebugGeometry(debugPoints);
    return std::dynamic_pointer_cast<DebugRenderGeometry>(debugPoints);
}

///
/// \brief This example demonstrates the cloth cutting simulation
/// using Position based dynamics and two haptic devices
///
int
main()
{
    auto sdk   = std::make_shared<SimulationManager>();
    auto scene = sdk->createNewScene("PBDPatternCutting");  

    //////////////////////////////////////////////////////////
    /// Phantom 1 -- cutter configurations  line108 ~ 184  ///
    //////////////////////////////////////////////////////////
    // Device clients
    auto client_1 = std::make_shared<HDAPIDeviceClient>(phantomOmni1Name);
    // Device Server
    auto server = std::make_shared<HDAPIDeviceServer>();
    server->addDeviceClient(client_1);
    sdk->addModule(server);
    // Device tracker
    auto deviceTracker_1 = std::make_shared<DeviceTracker>(client_1);
    deviceTracker_1->setTranslationOffset(Vec3d(4, 1, 4));
    deviceTracker_1->setTranslationScaling(0.1);
    Quatd rot(sqrt(2) / 2, 0, -sqrt(2) / 2, 0);
    deviceTracker_1->setRotationOffset(rot);

    //read the obj for CutterMesh
    auto CutterMesh = MeshIO::read(iMSTK_DATA_ROOT "/laptool/simpleTool.obj");
    if (!CutterMesh) 
        std::cout << "Can not read CutterMesh...\n";
    //CutterMesh->scale(0.2);
    auto cutterUpperJawMesh = MeshIO::read(iMSTK_DATA_ROOT "/laptool/endoShears_upper.obj");
    if (!cutterUpperJawMesh)
        std::cout << "Can not read cutterUpperJawMesh...\n";
    //cutterUpperJawMesh->rotate(Vec3d(0, 1., 0), -PI / 2, Geometry::TransformType::ApplyToData);
    cutterUpperJawMesh->scale(0.5);
    auto cutterLowerJawMesh = MeshIO::read(iMSTK_DATA_ROOT "/laptool/endoShears_lower.obj");
    if (!cutterLowerJawMesh)
        std::cout << "Can not read cutterLowerJawMesh...\n";
    //cutterLowerJawMesh->rotate(Vec3d(0, 1., 0), -PI / 2, Geometry::TransformType::ApplyToData);
    cutterLowerJawMesh->scale(0.5);
    

    //Create the tool object
    auto cutter = std::make_shared<PbdObject>("cutterObject");
    cutter->setCollidingGeometry(CutterMesh);
    cutter->setPhysicsGeometry(CutterMesh);
    auto toolMaterial = std::make_shared<RenderMaterial>();
    toolMaterial->setDisplayMode(RenderMaterial::DisplayMode::/*WIREFRAME_*/SURFACE);
    auto cuttingToolTexture = std::make_shared<Texture>(iMSTK_DATA_ROOT "/laptool/metal.jpg", Texture::Type::DIFFUSE);
    toolMaterial->addTexture(cuttingToolTexture);
    auto toolVisualModel = std::make_shared<VisualModel>(CutterMesh);
    toolVisualModel->setRenderMaterial(toolMaterial);
    cutter->addVisualModel(toolVisualModel);

    //Create the model / object for upper jaw
    auto toolupper = std::make_shared<VisualObject>("toolUpper");
    auto toolUpperVisualModel = std::make_shared<VisualModel>(cutterUpperJawMesh);
    toolUpperVisualModel->setRenderMaterial(toolMaterial);
    toolupper->addVisualModel(toolUpperVisualModel);

    // configure tool model
    auto toolpbdParams = std::make_shared<PBDModelConfig>();
    toolpbdParams->m_uniformMassValue = 0.0; // 0 means non-moving objects
    toolpbdParams->m_proximity = 0.3; // not used in CD or CH at all
    toolpbdParams->m_contactStiffness = 1.0;

    //Create the object for lower jaw
    auto toollower = std::make_shared<VisualObject>("toolLower");
    auto toolLowerVisualModel = std::make_shared<VisualModel>(cutterLowerJawMesh);
    toolLowerVisualModel->setRenderMaterial(toolMaterial);
    toollower->addVisualModel(toolLowerVisualModel);

    //Set the PBD model of the tool
    auto cutterModel = std::make_shared<PbdModel>();
    cutterModel->setModelGeometry(CutterMesh);
    cutterModel->configure(toolpbdParams);
    cutter->setDynamicalModel(cutterModel);

    // Add cutter object  in the scene.
    scene->addSceneObject(cutter);
    scene->addSceneObject(toolupper);
    scene->addSceneObject(toollower);

    // Create and add tool object controller in the scene
    auto lapController = std::make_shared<LaparoscopicToolController>(cutter, toolupper, toollower, deviceTracker_1);
    scene->addObjectController(lapController);

    /////////////////////////////////////////////////
    /// Phantom 2 -- Grasping Tool Configurations ///
    /////////////////////////////////////////////////
    auto client_2 = std::make_shared<HDAPIDeviceClient>(phantomOmni2Name);
    server->addDeviceClient(client_2);
    auto deviceTracker_2 = std::make_shared<DeviceTracker>(client_2);
    deviceTracker_2->setTranslationOffset(Vec3d(3, 1, 4));
    deviceTracker_2->setTranslationScaling(0.08);
    deviceTracker_2->setRotationOffset(rot);
    //read the obj for Grasper Mesh
    auto GrasperMesh = MeshIO::read(iMSTK_DATA_ROOT "/laptool/simpleTool.obj");
    if (!GrasperMesh)
        std::cout << "Can not read GrasperMesh...\n";

    auto GrasperUpperJawMesh = MeshIO::read(iMSTK_DATA_ROOT "/laptool/upper.obj");
    if (!GrasperUpperJawMesh)
        std::cout << "Can not read GrasperUpperJawMesh...\n";
    GrasperUpperJawMesh->rotate(Vec3d(0, 0, 1.0), PI / 2, Geometry::TransformType::ApplyToData);
    //GrasperUpperJawMesh->scale(0.04);

    auto GrasperLowerJawMesh = MeshIO::read(iMSTK_DATA_ROOT "/laptool/lower.obj");
    if (!GrasperLowerJawMesh)
        std::cout << "Can not read GrasperLowerJawMesh...\n";
    GrasperLowerJawMesh->rotate(Vec3d(0, 0, 1.0), PI / 2, Geometry::TransformType::ApplyToData);
    //GrasperLowerJawMesh->scale(0.04);

    //Create the grasper object
    auto grasper = std::make_shared<PbdObject>("grasperObject");
    grasper->setCollidingGeometry(GrasperMesh);
    grasper->setPhysicsGeometry(GrasperMesh);
    auto grasperVisualModel = std::make_shared<VisualModel>(GrasperMesh);
    grasperVisualModel->setRenderMaterial(toolMaterial);
    grasper->addVisualModel(grasperVisualModel);

    //Create the model / object for upper jaw
    auto grasperupper = std::make_shared<VisualObject>("grasperUpper");
    auto grasperUpperVisualModel = std::make_shared<VisualModel>(GrasperUpperJawMesh);
    grasperUpperVisualModel->setRenderMaterial(toolMaterial);
    grasperupper->addVisualModel(grasperUpperVisualModel);

    //Create the object for lower jaw
    auto grasperlower = std::make_shared<VisualObject>("grasperLower");
    auto grasperLowerVisualModel = std::make_shared<VisualModel>(GrasperLowerJawMesh);
    grasperLowerVisualModel->setRenderMaterial(toolMaterial);
    grasperlower->addVisualModel(grasperLowerVisualModel);

    //Set the PBD model of the tool
    auto grasperModel = std::make_shared<PbdModel>();
    grasperModel->setModelGeometry(GrasperMesh);
    grasperModel->configure(toolpbdParams);
    grasper->setDynamicalModel(grasperModel);

    //// Add grasper object  in the scene.
    scene->addSceneObject(grasper);
    scene->addSceneObject(grasperupper);
    scene->addSceneObject(grasperlower);

    //// Create and add tool object controller in the scene
    auto grasperController = std::make_shared<LaparoscopicToolController>(grasper, grasperupper, grasperlower, deviceTracker_2);
    scene->addObjectController(grasperController);

    //  Cylinder to indicate the cutting blade
    auto CylinderGeom = std::make_shared<Cylinder>();
    CylinderGeom->setRadius(0.05);
    CylinderGeom->setLength(1.);

    auto CylinderObj = std::make_shared<VisualObject>("Cylinder");
    CylinderObj->setVisualGeometry(CylinderGeom);
    scene->addSceneObject(CylinderObj);

    /////////////////////////////////////////////////
    ///   Create surface mesh -- Cloth Modeling   ///
    /////////////////////////////////////////////////
    StdVectorOfVec3d vertList;
    const double width = 10.0;
    const double height = 10.0;
    const int nRows = 21;
    const int nCols = 21;
    vertList.resize(nRows * nCols);
    const double dy = width / (double)(nCols - 1);
    const double dx = height / (double)(nRows - 1);
    for (int i = 0; i < nRows; ++i)
    {
        for (int j = 0; j < nCols; j++)
        {
            vertList[i * nCols + j] = Vec3d((double)dx * i, 1.0, (double)dy * j);
        }
    }

    //auto gauzeMesh = MeshIO::read(iMSTK_DATA_ROOT "/patternCutting/gauze_20x20.obj");
    //if (!gauzeMesh)
    //    std::cout << "Can not read gauzeMesh...\n";
    //if (gauzeMesh = std::dynamic_pointer_cast<SurfaceCuttingManager>(gauzeMesh))
    //    std::cout << "CuttingManager for gauze created!\n";

    // Add connectivity data
    std::vector<SurfaceMesh::TriangleArray> triangles;
    for (std::size_t i = 0; i < nRows - 1; ++i)
    {
        for (std::size_t j = 0; j < nCols - 1; j++)
        {
            SurfaceMesh::TriangleArray tri[2];
            //fliped version below, make sure verts lay in counterclockwise order
            tri[0] = { { i * nCols + j + 1 , (i + 1) * nCols + j, i*nCols + j} };
            tri[1] = { { (i + 1) * nCols + j, i * nCols + j + 1, (i + 1) * nCols + j + 1 } };
            triangles.push_back(tri[0]);
            triangles.push_back(tri[1]);
        }
    }
    auto surfMesh = std::make_shared<SurfaceCuttingManager>();
    //auto surfMesh = std::dynamic_pointer_cast<SurfaceCuttingManager>(gauzeMesh);
    //surfMesh->initializeTriangles(surfMesh->getTrianglesVertices());
    surfMesh->setInitialVertexPositions(vertList);
    surfMesh->setVertexPositions(vertList);
	surfMesh->initializeTriangles(triangles);
    
    // Set UV coordinates
    StdVectorOfVectorf UVs(surfMesh->getNumVertices());
    for (unsigned int i = 0; i < surfMesh->getNumVertices(); i++)
    {
        Vectorf UV(2);
        UV[0] = surfMesh->getVertexPosition(i)[0]/10.0;
        UV[1] = surfMesh->getVertexPosition(i)[2]/10.0;
        UVs[i] = UV;
    }
    surfMesh->setDefaultTCoords("tCoords");
    surfMesh->setPointDataArray("tCoords", UVs);

    // Create Cloth Object & Model
    auto clothObj = std::make_shared<PbdObject>("Cloth");
    auto clothModel = std::make_shared<PbdModel>();
    clothModel->setModelGeometry(surfMesh);

    // configure model
    auto pbdParams = std::make_shared<PBDModelConfig>();

    // Constraints
    pbdParams->enableConstraint(PbdConstraint::Type::Distance, 0.8);
    pbdParams->enableConstraint(PbdConstraint::Type::Dihedral, 0.01);
    pbdParams->m_fixedNodeIds = { 440, 439, 420, 438, 437, 436, 435, 434, 433, 432, 431, 430, 429, 428, 427, 426, 425, 424, 423, 422, 421, 420, 0,20, 1, 19 };
    pbdParams->m_contactStiffness = 1;

    // Other parameters
    pbdParams->m_uniformMassValue = 0.5;
    pbdParams->m_gravity          = Vec3d(0, -9.8, 0);
    pbdParams->m_dt               = 0.02;
    pbdParams->m_maxIter          = 30;

    // Set the parameters
    clothModel->configure(pbdParams);
    clothObj->setDynamicalModel(clothModel);
    clothObj->setPhysicsGeometry(surfMesh);

    // Solver
    auto pbdSolver = std::make_shared<PbdSolver>();
    pbdSolver->setPbdObject(clothObj);
    scene->addNonlinearSolver(pbdSolver);

    //Collision Model
    clothObj->setCollidingGeometry(surfMesh);
    //Visual model for Cloth
    auto clothMaterial = std::make_shared<RenderMaterial>();
    auto clothTexture = std::make_shared<Texture>(iMSTK_DATA_ROOT "/patternCutting/gauze_diffuse.jpg", Texture::Type::DIFFUSE);
    clothMaterial->addTexture(clothTexture);
    clothMaterial->setBackFaceCulling(false);
    clothMaterial->setColor(Color::LightGray);
    clothMaterial->setDisplayMode(RenderMaterial::DisplayMode::WIREFRAME_SURFACE);
    auto surfMeshModel = std::make_shared<VisualModel>(surfMesh);
    surfMeshModel->setRenderMaterial(clothMaterial);
    clothObj->addVisualModel(surfMeshModel);

    // Create a plane in the scene
    auto planeGeom = std::make_shared<Plane>();
    planeGeom->setWidth(30);
    planeGeom->setPosition(5.0, -1.0, 5.0);
    auto planeObj = std::make_shared<CollidingObject>("Plane");
    planeObj->setVisualGeometry(planeGeom);
    planeObj->setCollidingGeometry(planeGeom);
    scene->addSceneObject(planeObj);
    // Create the cloth_plane collision pair
    auto planeColData = std::make_shared<CollisionData>();
    auto planeCD = std::make_shared<PointSetToPlaneCD>(surfMesh, planeGeom, planeColData);
    // object1 = object2 below since object 2 is not used at all
    auto planeCH = std::make_shared<PBDCollisionHandling>(CollisionHandling::Side::A, planeColData, clothObj, clothObj, pbdSolver); 
    auto cloth_plane_pair = scene->getCollisionGraph()->addInteractionPair(clothObj, planeObj,
        planeCD, planeCH, nullptr);
    

    //Add cutter cloth iteraction pair
    auto cutterColData = std::make_shared<CollisionData>();
    auto CD = std::make_shared<MeshToMeshBruteForceCD>( CutterMesh, surfMesh, cutterColData);
    auto CH = std::make_shared<PBDCollisionHandling>(CollisionHandling::Side::A,
        CD->getCollisionData(), cutter, clothObj, pbdSolver);
    std::shared_ptr<ToolSurfaceInteractionPair> cutter_pair = scene->getCollisionGraph()->addToolSurfaceInteractionPair(cutter, clothObj, CD, CH, nullptr);
    cutter_pair->setDeviceClient(client_1);
    cutter_pair->setLapToolController(lapController);
    cutter_pair->setToolFunction(ToolState::ToolFunction::CUT);

    //Add grasper cloth iteraction pair
    auto grasperColData = std::make_shared<CollisionData>();
    auto CD2 = std::make_shared<MeshToMeshBruteForceCD>(GrasperMesh, surfMesh, grasperColData);
    auto CH2 = std::make_shared<PBDCollisionHandling>(CollisionHandling::Side::A,
        CD2->getCollisionData(), grasper, clothObj, pbdSolver);
    std::shared_ptr<ToolSurfaceInteractionPair> grasper_pair = scene->getCollisionGraph()->addToolSurfaceInteractionPair(grasper, clothObj, CD2, CH2, nullptr);
    grasper_pair->setDeviceClient(client_2);
    grasper_pair->setLapToolController(grasperController);
    grasper_pair->setToolFunction(ToolState::ToolFunction::GRASP);

   

    ////////////////////////////////
    /// Debugging Configurations ///
    ////////////////////////////////
    // Create debug Info
    auto debugTriangles = addTrianglesDebugRendering(scene, Color::Orange);
    auto debugLinesBroken = addLinesDebugRendering(scene, "DebugBrokenLines",Color::Green);
    auto debugLinesConnected = addLinesDebugRendering(scene, "DebugConnectedLines", Color::Red, 4.0);
    auto debugCuttingBlade = addLinesDebugRendering(scene, "DebugToolBlade", Color::Yellow);
    auto debugToolOrientation = addLinesDebugRendering(scene, "DebugToolOrientation", Color::Blue);
    auto debugPointsMeasured = addPointsDebugRendering(scene);
    // metrics
    int nbrPointsInsidePattern = 0;
    float avgAccuracy = 0;
    float patternRadius = 2.83;
    std::vector<float> distFromCutPointstoCenter(0);
    bool DebugModeOn = false;
    bool startedMeasuring = false; 

    // Light (white) DirectionalLight
    auto whiteLight1 = std::make_shared<DirectionalLight>("whiteLight1");
    whiteLight1->setFocalPoint(Vec3d(0, -1, 0));
    whiteLight1->setIntensity(1);

    // Light 2 (white)
    auto whiteLight2 = std::make_shared<SpotLight>("whiteLight2");
    whiteLight2->setPosition(Vec3d(9, 16, 5));
    whiteLight2->setFocalPoint(Vec3d(0, -15, 5));
    whiteLight2->setColor(Color::White);
    whiteLight2->setIntensity(90);
    whiteLight2->setSpotAngle(50);


    // Add in scene
    scene->addLight(whiteLight1);
    scene->addLight(whiteLight2);
    scene->addSceneObject(clothObj);

	//x deep, y up, z right
    scene->getCamera()->setFocalPoint(5, -1, 5);
    scene->getCamera()->setPosition(0, 15.0, 5.0);
  
    
    sdk->setActiveScene(scene);
    // Get the VTKViewer
    auto viewer = std::dynamic_pointer_cast<VTKViewer>(sdk->getViewer());
    viewer->getVtkRenderWindow()->SetSize(1920, 1080);

    auto statusManager = viewer->getTextStatusManager();
    statusManager->setStatusFontSize(VTKTextStatusManager::Custom, 25);
    statusManager->setStatusFontColor(VTKTextStatusManager::Custom, Color::Orange);

    // Get VTK Renderer
    auto renderer = std::dynamic_pointer_cast<VTKRenderer>(viewer->getActiveRenderer());
    LOG_IF(FATAL, (!renderer)) << "Invalid renderer: Only VTKRenderer is supported for debug rendering";

    // Update the debug data every frame from the cutting manager
    auto debugFunc =
        [&/*DebugModeOn, &CylinderGeom, &clothModel, &CutterMesh, &debugTriangles, &debugCuttingBlade, &totalDists, &avgAccuracy, &patternRadius, &debugToolOrientation,
        &viewer, &debugLinesConnected, &debugLinesBroken, &surfMesh, &renderer, &statusManager, &distFromCutPointstoCenter, &debugPoints, &cutter_pair, &startMeasuring*/](Module* module)
    {     
        surfMesh->m_TopologyLock.lock();
        std::shared_ptr<PbdState> initialState = clothModel->getInitialState();
        /*CylinderGeom->setTranslation(CutterMesh->getTranslation()+ CutterMesh->getRotation() * Vec3d(0, 0, -1)/2);
        CylinderGeom->setRotation(CutterMesh->getRotation());*/
        if (DebugModeOn)
        {
            debugToolOrientation->clear();
            std::shared_ptr<ToolState> state = cutter_pair->getToolState();
            debugToolOrientation->appendVertex(state->toolTipStartPos);
            debugToolOrientation->appendVertex(state->toolTipStartPos + state->toolDir);
            debugToolOrientation->appendVertex(state->toolTipStartPos);
            debugToolOrientation->appendVertex(state->toolTipStartPos + (state->toolDir).cross(state->cutPlaneNormal));
        }
        if (DebugModeOn && surfMesh->m_brokenEdges.size() > (debugLinesBroken->getNumVertices()/2.0 + debugLinesConnected->getNumVertices()/2.0) )
        {
            
            debugLinesConnected->clear();
            debugLinesBroken->clear();
            debugTriangles->clear();
            nbrPointsInsidePattern = 0;
            distFromCutPointstoCenter.resize(0);
            Vec3d center(5, 1, 5);
            for (size_t i = 0; i < surfMesh->m_brokenEdges.size(); i++)
            {
                if (surfMesh->m_brokenEdges[i].nodeId[0] < initialState->getPositions().size()
                    && surfMesh->m_brokenEdges[i].nodeId[1] < initialState->getPositions().size())
                {
                    debugCuttingBlade->appendVertex(surfMesh->d_toolPos[0]);
                    debugCuttingBlade->appendVertex(surfMesh->d_toolPos[1]);
                    if (surfMesh->m_brokenEdges[i].isConnected)
                    {
                        debugLinesConnected->appendVertex(initialState->getVertexPosition(surfMesh->m_brokenEdges[i].nodeId[0]));
                        debugLinesConnected->appendVertex(initialState->getVertexPosition(surfMesh->m_brokenEdges[i].nodeId[1]));
                    }
                    else
                    {
                        debugLinesBroken->appendVertex(initialState->getVertexPosition(surfMesh->m_brokenEdges[i].nodeId[0]));
                        debugLinesBroken->appendVertex(initialState->getVertexPosition(surfMesh->m_brokenEdges[i].nodeId[1]));
                    }
                    Vec3d cutPointPos = initialState->getVertexPosition(surfMesh->m_brokenEdges[i].nodeId[0])* (1 - surfMesh->m_brokenEdges[i].brokenCoord)
                        + initialState->getVertexPosition(surfMesh->m_brokenEdges[i].nodeId[1]) * surfMesh->m_brokenEdges[i].brokenCoord;
                    if (!startedMeasuring && (cutPointPos - center).norm() < 3.18) // start measuring the accuracy after the cut has entered the pattern
                    {
                        startedMeasuring = true;    
                    }
                    if (startedMeasuring)
                    {
                        //debugPointsMeasured->appendVertex(cutPointPos);
                        distFromCutPointstoCenter.push_back((cutPointPos - center).norm()); 
                    }

                }
            }
            for (auto k = 0; k < distFromCutPointstoCenter.size(); k++)
            {
                if (distFromCutPointstoCenter[k] > 2.48 && distFromCutPointstoCenter[k] < 3.18)
                    nbrPointsInsidePattern++; /*abs(distFromCutPointstoCenter[k] - patternRadius);*/
            }
            if (nbrPointsInsidePattern != 0)
                avgAccuracy = (float)nbrPointsInsidePattern / distFromCutPointstoCenter.size();
        }   
       
        if (DebugModeOn && surfMesh->d_listUncarvableTris.size() > debugTriangles->getNumVertices() / 3.0)
        {
            auto & list = surfMesh->d_listUncarvableTris;
            for (size_t i = 0; i < list.size(); i++)
            {
                size_t numInitVerts = initialState->getPositions().size();
                if (list[i].at(0) < numInitVerts && list[i].at(1) < numInitVerts && list[i].at(2) < numInitVerts)
                {
                    debugTriangles->appendVertex(initialState->getVertexPosition(list[i].at(0)));
                    debugTriangles->appendVertex(initialState->getVertexPosition(list[i].at(1)));
                    debugTriangles->appendVertex(initialState->getVertexPosition(list[i].at(2)));
                }
            }
        }
        if (!DebugModeOn) {
            debugLinesConnected->clear();
            debugLinesBroken->clear();
            debugTriangles->clear();
            debugCuttingBlade->clear();
            debugToolOrientation->clear();
        }
        debugLinesBroken->setDataModified(true);
        debugLinesConnected->setDataModified(true);
        debugTriangles->setDataModified(true);
        debugCuttingBlade->setDataModified(true);
        debugToolOrientation->setDataModified(true);
        //debugPoints->setDataModified(true);
        for (auto& delegate : renderer->getDebugRenderDelegates())
        {
            if (delegate != nullptr)
                delegate->updateDataSource();
        }
        
        statusManager->setCustomStatus("Mesh Info: " +
            std::to_string(debugLinesBroken->getNumVertices() / 2) + " (broken edges)  | " +
            std::to_string(surfMesh->getNumTriangles()) + " (triangles) | " +
            " Accuracy: " + std::to_string(avgAccuracy*100) +"% "
        );
        surfMesh->m_TopologyLock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    };
    sdk->getSceneManager(scene)->setPostUpdateCallback(debugFunc);

    viewer->setOnCharFunction('b', [&](InteractorStyle* c) -> bool
    {  
        client_2->setButton(0, false);
        client_2->setButton(1, true);
        return false;
    });
    viewer->setOnCharFunction('n', [&](InteractorStyle* c) -> bool
    {
        client_2->setButton(1, false);
        client_2->setButton(0, true);
        return false;
    });
    viewer->setOnCharFunction('m', [&](InteractorStyle* c) -> bool
    {
        DebugModeOn = !DebugModeOn;
        return DebugModeOn;
    });
    // Start
    sdk->startSimulation(SimulationStatus::RUNNING);

    return 0;
}
