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

#include "imstkMath.h"
#include "imstkTimer.h"
#include "imstkSimulationManager.h"

// Objects
#include "imstkVirtualCouplingPBDObject.h"
#include "imstkDynamicObject.h"
#include "imstkSceneObject.h"
#include "imstkLight.h"
#include "imstkCamera.h"

#include "imstkGraph.h"

// Geometry
#include "imstkSphere.h"
#include "imstkCylinder.h"
#include "imstkTetrahedralMesh.h"
#include "imstkSurfaceMesh.h"
#include "imstkMeshIO.h"
#include "imstkLineMesh.h"
#include <Eigen/Geometry>

// Devices and controllers
#include "imstkHDAPIDeviceClient.h"
#include "imstkHDAPIDeviceServer.h"
#include "imstkVRPNDeviceClient.h"
#include "imstkVRPNDeviceServer.h"
#include "imstkSceneObjectController.h"

// Collisions
#include "imstkInteractionPair.h"
#include "imstkPointSetToSphereCD.h"
#include "imstkPointSetToSawCD.h"
#include "imstkVirtualCouplingCH.h"
#include "imstkBoneDrillingCH.h"
#include "imstkBoneSawingCH.h"
#include "imstkMultiMeshBoneSawingCH.h"

// logger
#include "g3log/g3log.hpp"
#include "imstkLogger.h"

// imstk utilities
#include "imstkAPIUtilities.h"

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

#ifdef iMSTK_AUDIO_ENABLED
#include <SFML/Audio.hpp>
#endif

using namespace imstk;

// settings
const bool useSawTool = true;
const bool displayCollPrimitives = false;
const bool useMandiblePartitions = true;
const bool displayInVRMode = false;

// Global variables
const std::string phantomOmniName = "Phantom1";
const std::string mandibleWholeFileName = iMSTK_DATA_ROOT "/orthognatic/mandible.veg";
const std::string maxillaFileName = iMSTK_DATA_ROOT "/orthognatic/maxilla.veg";
const std::string sawFileName = iMSTK_DATA_ROOT "/orthognatic/saw.obj";
const std::string soundFileName = iMSTK_DATA_ROOT "/orthognatic/Dentistdrill3.wav";
const std::string mandibleLeftRightFileName[2] = { iMSTK_DATA_ROOT "/orthognatic/Mandible-left-surface.stl",
iMSTK_DATA_ROOT "/orthognatic/Mandible-right-surface.stl" };

// partitions
const unsigned int numDivisions[3] = { 4, 20, 20 };
const string mandibleCenterFileName(iMSTK_DATA_ROOT "/orthognatic/center.vtu");
std::vector<std::shared_ptr<TetrahedralMesh>> partitionedMeshes;

// scene and simulation manager
std::shared_ptr<SimulationManager> sdk;
std::shared_ptr<Scene> scene;

// Device
#ifdef iMSTK_USE_OPENHAPTICS
std::shared_ptr<HDAPIDeviceClient> client;
std::shared_ptr<HDAPIDeviceServer> server;
#endif
std::shared_ptr<DeviceTracker> deviceTracker;
std::shared_ptr<SceneObjectController> sawController;
std::shared_ptr<SceneObjectController> burrController;

// Scene objects
std::vector<std::shared_ptr<CollidingObject>> mandibleCenterPartitions;
std::shared_ptr<CollidingObject> mandibleWhole;
std::shared_ptr<VisualObject> mandibleLeftRight[2];
std::shared_ptr<VisualObject> maxilla;
std::shared_ptr<CollidingObject> saw;
std::shared_ptr<CollidingObject> burr;

// collision primitives
std::shared_ptr<OBB> obb;
std::shared_ptr<OBB> bb;
std::shared_ptr<SurfaceMesh> obbSurfMesh;
std::shared_ptr<SurfaceMesh> bbSurfMesh;

const double sawScale = 10.0;
const Mat3d rotationSaw = Mat3d::Identity();//Eigen::AngleAxis<double>(PI / 2, Vec3d(1., 0., 0.)).toRotationMatrix();//*Eigen::AngleAxis<double>(PI, Vec3d(0., 1., 0.)).toRotationMatrix();

void createDeviceClient()
{
#ifdef iMSTK_USE_OPENHAPTICS
    // Device clients
    client = std::make_shared<HDAPIDeviceClient>(phantomOmniName);

    // Device Server
    server = std::make_shared<HDAPIDeviceServer>();
    server->addDeviceClient(client);
    sdk->addModule(server);

    // Device tracker
    deviceTracker = std::make_shared<DeviceTracker>(client);
    deviceTracker->setTranslationOffset(Vec3d(50.0, .0, .0));
    //deviceTracker->setTranslationScaling(0.1);
#endif
}

void createMandible()
{
    // Create mandible scene object
    auto mandibleTetMesh = MeshIO::read(mandibleWholeFileName);
    if (!mandibleTetMesh)
    {
        LOG(WARNING) << "Could not read mandible mesh from file ";
        return;
    }
    //mandibleTetMesh->scale(0.1, Geometry::TransformType::ApplyToData);
    mandibleTetMesh->rotate(Vec3d(1., 0., 0.), -PI / 2., Geometry::TransformType::ApplyToData);
    mandibleTetMesh->rotate(Vec3d(0., 1., 0.), PI, Geometry::TransformType::ApplyToData);
    mandibleTetMesh->translate(Vec3d(0., 0., -55.), Geometry::TransformType::ApplyToData);

    mandibleWhole = std::make_shared<CollidingObject>("mandibleWhole");
    mandibleWhole->setCollidingGeometry(mandibleTetMesh);
    mandibleWhole->setVisualGeometry(mandibleTetMesh);

    auto materialMandible = std::make_shared<RenderMaterial>();
    materialMandible->setDiffuseColor(Color::Beige);
    mandibleTetMesh->setRenderMaterial(materialMandible);

    scene->addSceneObject(mandibleWhole);
           
}

void partitionUnstructuredTetMeshWithGrid()
{    
    const unsigned int numGridCells = numDivisions[0] * numDivisions[1] * numDivisions[2];      

    // Load the unstructured tetrahedral mesh
    auto originalMesh = MeshIO::read(mandibleCenterFileName);
    if (!originalMesh)
    {
        LOG(WARNING) << "Could not read tetrahedral mesh from file ";
        return;
    }
    auto tetMesh = std::static_pointer_cast<TetrahedralMesh>(originalMesh);   

    //-----

    Vec3d bbMin, bbMax;
    tetMesh->computeBoundingBox(bbMin, bbMax, 2.0);

    Vec3d spacing;
    for (int i = 0; i < 3; ++i)
    {
        spacing[i] = (bbMax[i] - bbMin[i]) / numDivisions[i];
    }

    // Assign each point in the original mesh the cell ID it belongs to
    std::vector<int> pointCellIds;
    for (auto& p : tetMesh->getVertexPositions())
    {
        auto index = Eigen::Vector3i(trunc((p[0] - bbMin[0]) / spacing[0]), trunc((p[1] - bbMin[1]) / spacing[1]), trunc((p[2] - bbMin[2]) / spacing[2]));// starts with 1
        pointCellIds.push_back(index[2] * numDivisions[0] * numDivisions[1] + index[1] * numDivisions[0] + index[0]);
    }

    // Based on the point cell IDs, assign each tetrahedra a cell ID
    std::vector<int> tetCellIds;
    for (auto& t : tetMesh->getTetrahedraVertices())
    {
        // all 4 IDs are equal
        if (pointCellIds[t[0]] == pointCellIds[t[1]] && pointCellIds[t[0]] == pointCellIds[t[2]] &&
            pointCellIds[t[0]] == pointCellIds[t[3]])
        {
            tetCellIds.push_back(pointCellIds[t[0]]);
        }
        // check if three IDs are equal
        else if (pointCellIds[t[0]] == pointCellIds[t[1]] && pointCellIds[t[1]] == pointCellIds[t[2]])
        {
            tetCellIds.push_back(pointCellIds[t[0]]);
        }
        else if (pointCellIds[t[0]] == pointCellIds[t[1]] && pointCellIds[t[1]] == pointCellIds[t[3]])
        {
            tetCellIds.push_back(pointCellIds[t[0]]);
        }
        else if (pointCellIds[t[0]] == pointCellIds[t[2]] && pointCellIds[t[2]] == pointCellIds[t[3]])
        {
            tetCellIds.push_back(pointCellIds[t[0]]);
        }
        else if (pointCellIds[t[1]] == pointCellIds[t[2]] && pointCellIds[t[2]] == pointCellIds[t[3]])
        {
            tetCellIds.push_back(pointCellIds[t[1]]);
        }
        // check if two IDs are equal
        else if (pointCellIds[t[0]] == pointCellIds[t[1]] || pointCellIds[t[0]] == pointCellIds[t[2]] ||
            pointCellIds[t[0]] == pointCellIds[t[3]])
        {
            tetCellIds.push_back(pointCellIds[t[0]]);
        }
        else if (pointCellIds[t[1]] == pointCellIds[t[2]] || pointCellIds[t[1]] == pointCellIds[t[3]])
        {
            tetCellIds.push_back(pointCellIds[t[1]]);
        }
        else if (pointCellIds[t[2]] == pointCellIds[t[3]])
        {
            tetCellIds.push_back(pointCellIds[t[2]]);
        }
        // all different
        else
        {
            tetCellIds.push_back(pointCellIds[t[0]]);
        }
    }

    //---

    // Renumber and add the nodes and elements and create mesh partitions
    for (size_t cellId = 0; cellId < numGridCells; cellId++)
    {
        auto gridPartition = std::make_shared<TetrahedralMesh>();
        std::map<size_t, size_t> oldNewNumberingMap;

        StdVectorOfVec3d points;
        std::vector<TetrahedralMesh::TetraArray> tetra;

        size_t tetIdGlobal = 0;
        size_t pointIdLocal = 0;
        for (const auto& t : tetMesh->getTetrahedraVertices())
        {
            if (tetCellIds[tetIdGlobal] == cellId)
            {
                TetrahedralMesh::TetraArray thisTet;
                for (int i = 0; i < 4; ++i)
                {
                    auto search = oldNewNumberingMap.find(t[i]);
                    if (search == oldNewNumberingMap.end())
                    {
                        oldNewNumberingMap[t[i]] = pointIdLocal;
                        thisTet[i] = pointIdLocal;

                        // add point
                        points.push_back(tetMesh->getVertexPosition(t[i]));

                        pointIdLocal++;
                    }
                    else
                    {
                        thisTet[i] = search->second;
                    }
                }

                // add the current element
                tetra.push_back(thisTet);
            }
            tetIdGlobal++;
        }

        if (points.size() != 0)
        {
            gridPartition->initialize(points, tetra, false);
            partitionedMeshes.push_back(gridPartition);
        }
    }

    if (0)
    {
        size_t partNum = 0;
        size_t maxNodes = 0;
        size_t maxElements = 0;

        LOG(INFO) << "---------------------------------";
        LOG(INFO) << "Partition    Verts    Elements";
        LOG(INFO) << "---------------------------------";
        for (const auto& parMesh : partitionedMeshes)
        {
            if (parMesh->getNumVertices() != 0)
            {
                auto nbNodes = parMesh->getMaxNumVertices();
                auto nbtet = parMesh->getNumTetrahedra();
                LOG(INFO) << partNum << "      " << nbNodes << "     " << nbtet;

                maxNodes = maxNodes < nbNodes ? nbNodes : maxNodes;
                maxElements = maxElements < nbtet ? nbtet : maxElements;
            }
            partNum++;
        }
        LOG(INFO) << "\nMax. Verts: " << maxNodes << "  Max. elements: " << maxElements;
        LOG(INFO) << "---------------------------------";
    }
}

void createMandibleWithPartitions()
{    
    // First create left and right pieces
    for (int i = 0; i < 2; ++i)
    {
        std::string name = "left";
        if (i == 1) { name = "right"; }

        // Create mandible scene object
        auto mandibleTetMesh = MeshIO::read(mandibleLeftRightFileName[i]);
        if (!mandibleTetMesh)
        {
            LOG(WARNING) << "Could not read mandible mesh from file for " << name << "piece";
            return;
        }
        mandibleTetMesh->scale(0.1, Geometry::TransformType::ApplyToData);
        mandibleTetMesh->rotate(Vec3d(1., 0., 0.), -PI / 2., Geometry::TransformType::ApplyToData);
        mandibleTetMesh->rotate(Vec3d(0., 1., 0.), PI, Geometry::TransformType::ApplyToData);
        mandibleTetMesh->translate(Vec3d(0., 20., -15.), Geometry::TransformType::ApplyToData);

        mandibleLeftRight[i] = std::make_shared<VisualObject>("mandible_" + name);
        mandibleLeftRight[i]->setVisualGeometry(mandibleTetMesh);

        auto materialMandible = std::make_shared<RenderMaterial>();
        materialMandible->setDiffuseColor(Color(0.96, 0.96, 0.8627, 1.0));
        mandibleTetMesh->setRenderMaterial(materialMandible);

        scene->addSceneObject(mandibleLeftRight[i]);
    }

    // add scene objects from the partitions of the center piece
    partitionUnstructuredTetMeshWithGrid();

    mandibleCenterPartitions.resize(partitionedMeshes.size());
    for (auto i = 0; i < partitionedMeshes.size(); ++i)
    {
        partitionedMeshes[i]->scale(0.1, Geometry::TransformType::ApplyToData);
        partitionedMeshes[i]->rotate(Vec3d(1., 0., 0.), -PI / 2., Geometry::TransformType::ApplyToData);
        partitionedMeshes[i]->rotate(Vec3d(0., 1., 0.), PI, Geometry::TransformType::ApplyToData);
        partitionedMeshes[i]->translate(Vec3d(0., 20., -15.), Geometry::TransformType::ApplyToData);

        mandibleCenterPartitions[i] = std::make_shared<CollidingObject>("mandibleCenterpart_" + std::to_string(i));
        mandibleCenterPartitions[i]->setVisualGeometry(partitionedMeshes[i]);
        mandibleCenterPartitions[i]->setCollidingGeometry(partitionedMeshes[i]);

        auto materialMandible = std::make_shared<RenderMaterial>();
        materialMandible->setDiffuseColor(Color::Beige);
        partitionedMeshes[i]->setRenderMaterial(materialMandible);

        scene->addSceneObject(mandibleCenterPartitions[i]);
    }
}

void createMaxilla()
{
    // Create maxilla scene object (visual)
    auto maxillaMesh = MeshIO::read(maxillaFileName);
    if (!maxillaMesh)
    {
        LOG(WARNING) << "Could not read maxilla mesh from file.";
        return;
    }
    auto maxillaGeom = std::dynamic_pointer_cast<SurfaceMesh>(maxillaMesh);
    maxilla = std::make_shared<VisualObject>("maxilla");
    maxilla->setVisualGeometry(maxillaGeom);
    scene->addSceneObject(maxilla);
}

void createSawTool()
{
    // Create saw tool scene Object
    auto sawMesh = MeshIO::read(sawFileName);
    auto sawGeom = std::dynamic_pointer_cast<SurfaceMesh>(sawMesh);
    sawGeom->rotate(rotationSaw, Geometry::TransformType::ApplyToData);
    sawGeom->scale(sawScale, Geometry::TransformType::ApplyToData);
    saw = std::make_shared<CollidingObject>("saw");
    saw->setVisualGeometry(sawGeom);


    auto sawMeshColl = MeshIO::read(sawFileName);
    auto sawGeomColl = std::dynamic_pointer_cast<SurfaceMesh>(sawMeshColl);
    sawGeomColl->rotate(rotationSaw, Geometry::TransformType::ApplyToData);
    sawGeomColl->scale(sawScale, Geometry::TransformType::ApplyToData);
    saw->setCollidingGeometry(sawGeomColl);

    scene->addSceneObject(saw);

#ifdef iMSTK_USE_OPENHAPTICS
    sawController = std::make_shared<SceneObjectController>(saw, deviceTracker);
    scene->addObjectController(sawController);
#endif

#ifdef iMSTK_AUDIO_ENABLED
    // Load a sound buffer from a .wav file
    sf::SoundBuffer buffer;
    if (!buffer.loadFromFile(soundFileName))
    {
        LOG(WARNING) << "createSawTool: Could not open the input sound file: " << soundFileName;
        return;
    }

    // Create a sound instance and play it
    sf::Sound sound(buffer);
    sound.setPosition(0., 0., 0.);
    sound.setMinDistance(5.);
    sound.setAttenuation(10.);
    sound.setLoop(true);

    sound.play();
    sound.pause();

#ifdef iMSTK_USE_OPENHAPTICS
    auto triggerSoundFunc =
        [&sound](Module* module)
        {
            auto serverMod = dynamic_cast<HDAPIDeviceServer*>(module);
            if (serverMod)
            {
                // trigger this when cutting the bone
                float pitch = serverMod->getDeviceClient(0)->getButton(1) ? 0.5 : 1.0;
                sound.setPitch(pitch);

                if (serverMod->getDeviceClient(0)->getButton(0))
                {
                    if (sound.getStatus() != sf::Sound::Playing)
                    {
                        sound.play();
                    }
                }
                else
                {
                    if (sound.getStatus() != sf::Sound::Paused)
                    {
                        sound.pause();
                    }
                }
            }
        };
    server->setPostUpdateCallback(triggerSoundFunc);

#endif // iMSTK_USE_OPENHAPTICS

#endif // iMSTK_AUDIO_ENABLED
}

void createBurrTool()
{
    // Create a virtual coupling object: Burr tool
    auto burrVisualGeom = std::make_shared<Sphere>();
    burrVisualGeom->setRadius(3.);
    auto burrCollidingGeom = std::make_shared<Sphere>();
    burrCollidingGeom->setRadius(3.);

    burr = std::make_shared<CollidingObject>("Burr");
    burr->setCollidingGeometry(burrCollidingGeom);
    burr->setVisualGeometry(burrVisualGeom);
    scene->addSceneObject(burr);

#ifdef iMSTK_USE_OPENHAPTICS
    burrController = std::make_shared<SceneObjectController>(burr, deviceTracker);
    scene->addObjectController(burrController);
#endif
}

void displayCollisionPrimitives()
{
    obb = std::make_shared<OBB>();
    bb = std::make_shared<OBB>();
    if (useSawTool)
    {
        obb->m_center = sawScale*rotationSaw*Vec3d(0.1, 0., -6.5);
        obb->m_halfLengths = sawScale*Vec3d(0.2/5, 0.1/20, 1.0);
        obb->m_axis[0] = rotationSaw*Vec3d(1., 0., 0.);
        obb->m_axis[1] = rotationSaw*Vec3d(0., 1., 0.);
        obb->m_axis[2] = rotationSaw*Vec3d(0., 0., 1.);

        // points
        obbSurfMesh = std::make_shared<SurfaceMesh>();
        StdVectorOfVec3d&& corners = obb->getCorners();
        obbSurfMesh->setInitialVertexPositions(corners);
        obbSurfMesh->setVertexPositions(corners);

        // connectivity
        std::vector<SurfaceMesh::TriangleArray> triangles = { { 0, 1, 2 },{ 0, 2, 3 },{ 1, 5, 6 },
        { 1, 6, 2 },{ 0, 1, 5 },{ 0, 5, 4 },{ 0, 4, 7 },{ 0, 7, 3 },
        { 3, 2, 6 },{ 3, 6, 7 },{ 4, 5, 6 },{ 4, 6, 7 } };

        obbSurfMesh->setTrianglesVertices(triangles);

        auto material = std::make_shared<RenderMaterial>();
        material->setDisplayMode(RenderMaterial::DisplayMode::WIREFRAME);
        material->setLineWidth(1.5);
        material->setDebugColor(Color::Red);
        obbSurfMesh->setRenderMaterial(material);

        auto obbObj = std::make_shared<VisualObject>("OBB");
        obbObj->setVisualGeometry(obbSurfMesh);

        if (displayCollPrimitives)
        {
            scene->addSceneObject(obbObj);
        }

        //-----

        bb->m_center = sawScale*rotationSaw*Vec3d(0., 0., -6.5);
        bb->m_halfLengths = sawScale*Vec3d(0.2, 0.1, 1.0);
        bb->m_axis[0] = rotationSaw*Vec3d(1., 0., 0.);
        bb->m_axis[1] = rotationSaw*Vec3d(0., 1., 0.);
        bb->m_axis[2] = rotationSaw*Vec3d(0., 0., 1.);

        // points
        bbSurfMesh = std::make_shared<SurfaceMesh>();
        StdVectorOfVec3d&& bb_corners = bb->getCorners();
        bbSurfMesh->setInitialVertexPositions(bb_corners);
        bbSurfMesh->setVertexPositions(bb_corners);

        // connectivity
        std::vector<SurfaceMesh::TriangleArray> bb_triangles = { { 0, 1, 2 },{ 0, 2, 3 },{ 1, 5, 6 },
        { 1, 6, 2 },{ 0, 1, 5 },{ 0, 5, 4 },{ 0, 4, 7 },{ 0, 7, 3 },
        { 3, 2, 6 },{ 3, 6, 7 },{ 4, 5, 6 },{ 4, 6, 7 } };

        bbSurfMesh->setTrianglesVertices(bb_triangles);

        auto bb_material = std::make_shared<RenderMaterial>();
        bb_material->setDisplayMode(RenderMaterial::DisplayMode::WIREFRAME);
        bb_material->setLineWidth(1.5);
        bb_material->setDebugColor(Color::Yellow);
        bbSurfMesh->setRenderMaterial(bb_material);

        auto bbObj = std::make_shared<VisualObject>("BB");
        bbObj->setVisualGeometry(bbSurfMesh);
        if (displayCollPrimitives)
        {
            scene->addSceneObject(bbObj);
        }

#ifdef iMSTK_USE_OPENHAPTICS
        if (displayCollPrimitives)
        {
            auto obbController = std::make_shared<SceneObjectController>(obbObj, deviceTracker);
            scene->addObjectController(obbController);
        }

#endif
        // Create colliding cylinder scene object
        auto CylinderGeomVis = std::make_shared<Cylinder>();
        CylinderGeomVis->setRadius(sawScale*0.2236);
        CylinderGeomVis->setLength(sawScale*2.);
        CylinderGeomVis->rotate(rotationSaw, Geometry::TransformType::ApplyToData);
        CylinderGeomVis->rotate(Vec3d(1., 0., 0.), PI / 2., Geometry::TransformType::ApplyToData);
        CylinderGeomVis->translate(rotationSaw*Vec3d(0., 0., sawScale* -6.5), Geometry::TransformType::ApplyToData);

        auto cylinderMaterial = std::make_shared<RenderMaterial>();
        cylinderMaterial->setDisplayMode(RenderMaterial::DisplayMode::WIREFRAME);
        cylinderMaterial->setLineWidth(1.5);
        cylinderMaterial->setDebugColor(Color::Green);
        CylinderGeomVis->setRenderMaterial(cylinderMaterial);

        auto CylinderObj = std::make_shared<CollidingObject>("Cylinder");
        CylinderObj->setVisualGeometry(CylinderGeomVis);
        CylinderObj->setCollidingGeometry(CylinderGeomVis);

        if (displayCollPrimitives)
        {
            scene->addSceneObject(CylinderObj);
        }

#ifdef iMSTK_USE_OPENHAPTICS
        if (displayCollPrimitives)
        {
            // Create and add virtual coupling object controller in the scene
            auto cylinderController = std::make_shared<SceneObjectController>(CylinderObj, deviceTracker);
            scene->addObjectController(cylinderController);
        }
#endif
    }
}

void orthognathicSurgery()
{
    // SDK and Scene
    sdk = std::make_shared<SimulationManager>(displayInVRMode);
    scene = sdk->createNewScene("orthognathicSurgery");

    createDeviceClient();
    if (useMandiblePartitions)
    {
        createMandibleWithPartitions();
    }
    else
    {
        createMandible();
    }
    //createMaxilla();

    if (useSawTool)
    {
        createSawTool();
    }
    else
    {
        createBurrTool();
    }

    displayCollisionPrimitives();

    // Create a collision graph
    auto graph = scene->getCollisionGraph();

    if (useSawTool)
    {
        if (useMandiblePartitions && saw)
        {            
            std::vector<std::shared_ptr<PointSet>> pointsMandible;
            for (auto i = 0; i < partitionedMeshes.size(); ++i)
            {
                pointsMandible.push_back(std::dynamic_pointer_cast<PointSet>(partitionedMeshes[i]));
            }

            CollisionData colData;
            auto CD = std::make_shared<PointSetToSawCD>(pointsMandible, deviceTracker, obb, bbSurfMesh, colData);
            auto CHA = std::make_shared<MultiMeshBoneSawingCH>(CollisionHandling::Side::A, colData, mandibleCenterPartitions, saw);
            graph->addInteractionPair(mandibleCenterPartitions[0], saw, CD, nullptr, CHA);
        }
        else
        {
            if (mandibleWhole && saw)
            {
                CollisionData colData;
                std::vector<std::shared_ptr<PointSet>> pointsMandible;
                pointsMandible.push_back(std::dynamic_pointer_cast<PointSet>(mandibleWhole->getCollidingGeometry()));

                auto CD = std::make_shared<PointSetToSawCD>(pointsMandible, deviceTracker, obb, bbSurfMesh, colData);
                auto CHA = std::make_shared<BoneSawingCH>(CollisionHandling::Side::A, colData, mandibleWhole, saw);
                graph->addInteractionPair(mandibleWhole, saw, CD, nullptr, CHA);
            }
        }        
    }
    else
    {
        if (mandibleWhole && burr)
        {
            auto pair = graph->addInteractionPair(mandibleWhole,
                burr,
                CollisionDetection::Type::PointSetToSphere,
                CollisionHandling::Type::BoneDrilling,
                CollisionHandling::Type::None);
        }
    }

    // Lights
    auto light1 = std::make_shared<DirectionalLight>("light1");
    light1->setFocalPoint(Vec3d(5, -8, -5));
    light1->setIntensity(0.6);

    auto light2 = std::make_shared<DirectionalLight>("light2");
    light2->setFocalPoint(Vec3d(-5, -8, -5));
    light2->setIntensity(0.6);

    scene->addLight(light1);
    scene->addLight(light2);

    // Camera
    auto camera = scene->getCamera();
    camera->setPosition(Vec3d(0, 10, 70));
    camera->setFocalPoint(Vec3d(0., 0., 0.));

    // Action
    sdk->setActiveScene(scene);
    sdk->startSimulation(false, true);
}

int main()
{
    std::cout << "-----------------------------------------\n"
              << "Starting Orthognathic surgery simulator\n"
              << "-----------------------------------------\n";

    orthognathicSurgery();

    return 0;
}
