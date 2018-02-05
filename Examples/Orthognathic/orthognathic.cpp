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

const bool useSawTool = true;
const bool displayCollStructures = false;

// Global variables
const std::string phantomOmniName = "Phantom1";
const std::string mandibleFileName = iMSTK_DATA_ROOT "/orthognatic/mandible.veg";
//const std::string mandibleFileName = iMSTK_DATA_ROOT "/orthognatic/ss.vtu";
const std::string maxillaFileName = iMSTK_DATA_ROOT "/orthognatic/maxilla.veg";
const std::string sawFileName = iMSTK_DATA_ROOT "/orthognatic/saw.obj";
const std::string soundFileName = iMSTK_DATA_ROOT "/orthognatic/Dentistdrill3.wav";

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
std::shared_ptr<CollidingObject> mandible;
std::shared_ptr<VisualObject> maxilla;
std::shared_ptr<CollidingObject> saw;
std::shared_ptr<CollidingObject> burr;

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
    //deviceTracker->setTranslationScaling(0.1);
#endif
}

void createSkull()
{
    // Create mandible scene object
    auto mandibleTetMesh = MeshIO::read(mandibleFileName);
    if (!mandibleTetMesh)
    {
        LOG(WARNING) << "Could not read mandible mesh from file ";
        return;
    }
    //mandibleTetMesh->scale(0.1, Geometry::TransformType::ApplyToData);
    mandibleTetMesh->rotate(Vec3d(1., 0., 0.), -PI/2., Geometry::TransformType::ApplyToData);
    mandibleTetMesh->rotate(Vec3d(0., 1., 0.), PI, Geometry::TransformType::ApplyToData);
    mandibleTetMesh->translate(Vec3d(0., 0., -55.), Geometry::TransformType::ApplyToData);
    mandible = std::make_shared<CollidingObject>("mandible");
    mandible->setCollidingGeometry(mandibleTetMesh);
    mandible->setVisualGeometry(mandibleTetMesh);

    auto materialMandible = std::make_shared<RenderMaterial>();
    materialMandible->setDiffuseColor(Color::Beige);
    mandibleTetMesh->setRenderMaterial(materialMandible);


    scene->addSceneObject(mandible);

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

#ifdef iMSTK_AUDIO_ENABLED_
    // Load a sound buffer from a .wav file
    sf::SoundBuffer buffer;
    if (!buffer.loadFromFile(soundFileName))
    {
        LOG(WARNING) << "testSound: Could not open the input sound file: " << soundFileName;
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

void orthognathicSurgery()
{
    // SDK and Scene
    sdk = std::make_shared<SimulationManager>();
    scene = sdk->createNewScene("orthognathicSurgery");

    createDeviceClient();
    createSkull();

    if (useSawTool)
    {
        createSawTool();
    }
    else
    {
        createBurrTool();
    }

    auto obb = std::make_shared<OBB>();
    auto bb = std::make_shared<OBB>();
    std::shared_ptr<SurfaceMesh> obbSurfMesh;
    std::shared_ptr<SurfaceMesh> bbSurfMesh;
    if (useSawTool)
    {
        obb->m_center = sawScale*rotationSaw*Vec3d(0., 0., -6.5);
        obb->m_halfLengths = sawScale*Vec3d(0.2, 0.1, 1.0);
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

        if (displayCollStructures)
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
        if (displayCollStructures)
        {
            scene->addSceneObject(bbObj);
        }

#ifdef iMSTK_USE_OPENHAPTICS
        if (displayCollStructures)
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

        if (displayCollStructures)
        {
            scene->addSceneObject(CylinderObj);
        }

#ifdef iMSTK_USE_OPENHAPTICS
        if (displayCollStructures)
        {
            // Create and add virtual coupling object controller in the scene
            auto cylinderController = std::make_shared<SceneObjectController>(CylinderObj, deviceTracker);
            scene->addObjectController(cylinderController);
        }
#endif
    }

    // Create a collision graph
    auto graph = scene->getCollisionGraph();

    if (useSawTool)
    {
        if (mandible && saw)
        {
            CollisionData colData;
            auto pointsMandible = std::dynamic_pointer_cast<PointSet>(mandible->getCollidingGeometry());
            auto CD = std::make_shared<PointSetToSawCD>(pointsMandible, deviceTracker, obb, bbSurfMesh, colData);
            auto CHA = std::make_shared<BoneSawingCH>(CollisionHandling::Side::A, colData, mandible, saw);

            graph->addInteractionPair(mandible, saw, CD, nullptr, CHA);
        }
    }
    else
    {
        if (mandible && burr)
        {
            auto pair = graph->addInteractionPair(mandible,
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
