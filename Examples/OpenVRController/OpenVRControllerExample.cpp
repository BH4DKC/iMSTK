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
#include "imstkLight.h"
#include "imstkLogger.h"
#include "imstkMeshIO.h"
#include "imstkNew.h"
#include "imstkOpenVRDeviceClient.h"
#include "imstkRenderMaterial.h"
#include "imstkScene.h"
#include "imstkSceneManager.h"
#include "imstkSceneObject.h"
#include "imstkSceneObjectController.h"
#include "imstkSurfaceMesh.h"
#include "imstkVisualModel.h"
#include "imstkVTKOpenVRViewer.h"

using namespace imstk;

std::shared_ptr<SceneObject>
makeHandleObject()
{
    imstkNew<SceneObject> scalpelHandle("ScalpelHandle");
    auto                  toolHandleMesh = MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "/Surgical Instruments/Scalpel/Scalpel_Handle.obj");
    toolHandleMesh->translate(0.0, 0.0, 1.0, Geometry::TransformType::ApplyToData);
    toolHandleMesh->rotate(Vec3d(0.0, 1.0, 0.0), 3.14, Geometry::TransformType::ApplyToData);
    toolHandleMesh->rotate(Vec3d(1.0, 0.0, 0.0), -1.57, Geometry::TransformType::ApplyToData);
    toolHandleMesh->scale(0.06, Geometry::TransformType::ApplyToData);

    imstkNew<VisualModel>    toolHandleModel(toolHandleMesh);
    imstkNew<RenderMaterial> material;
    material->setDisplayMode(RenderMaterial::DisplayMode::Surface);
    material->setShadingModel(RenderMaterial::ShadingModel::PBR);
    material->setMetalness(0.9f);
    material->setRoughness(0.2f);
    material->addTexture(std::make_shared<Texture>(
                iMSTK_DATA_ROOT "/Surgical Instruments/Scalpel/Scalpel_Albedo.png",
                Texture::Type::Diffuse));
    toolHandleModel->setRenderMaterial(material);

    scalpelHandle->addVisualModel(toolHandleModel);

    return scalpelHandle;
}

std::shared_ptr<SceneObject>
makeBlade(std::string filename)
{
    imstkNew<SceneObject> scalpelBlade(filename);
    auto                  blade10Mesh = MeshIO::read<SurfaceMesh>(iMSTK_DATA_ROOT "/Surgical Instruments/Scalpel/" + filename + ".obj");
    blade10Mesh->translate(0.0, 0.0, 1.0, Geometry::TransformType::ApplyToData);
    blade10Mesh->rotate(Vec3d(0.0, 1.0, 0.0), 3.14, Geometry::TransformType::ApplyToData);
    blade10Mesh->rotate(Vec3d(1.0, 0.0, 0.0), -1.57, Geometry::TransformType::ApplyToData);
    blade10Mesh->scale(0.06, Geometry::TransformType::ApplyToData);

    imstkNew<VisualModel>    blade10Model(blade10Mesh);
    imstkNew<RenderMaterial> material;
    material->setDisplayMode(RenderMaterial::DisplayMode::Surface);
    material->setShadingModel(RenderMaterial::ShadingModel::PBR);
    material->setMetalness(0.9f);
    material->setRoughness(0.2f);
    material->addTexture(std::make_shared<Texture>(
                iMSTK_DATA_ROOT "/Surgical Instruments/Scalpel/Scalpel_Albedo.png",
                Texture::Type::Diffuse));
    blade10Model->setRenderMaterial(material);

    scalpelBlade->addVisualModel(blade10Model);
    return scalpelBlade;
}

///
/// \brief This example demonstrates rendering and controlling a SceneObject with OpenVR
/// as well as swapping a tool
///
int
main()
{
    // Write log to stdout and file
    Logger::startLogger();

    // Setup the scene
    imstkNew<Scene> scene("OpenVRControllerExample");

    std::shared_ptr<SceneObject> scalpelHandle = makeHandleObject();
    scene->addSceneObject(scalpelHandle);

    std::shared_ptr<SceneObject> scalpelBlade10 = makeBlade("Scalpel_Blade10");
    scene->addSceneObject(scalpelBlade10);

    std::shared_ptr<SceneObject> scalpelBlade15 = makeBlade("Scalpel_Blade15");
    scene->addSceneObject(scalpelBlade15);
    scalpelBlade15->getMasterGeometry()->setTranslation(0.2, 1.0, -0.8);

    // Lights
    imstkNew<DirectionalLight> dirLight("DirLight");
    dirLight->setIntensity(4);
    dirLight->setColor(Color(1.0, 0.95, 0.8));
    dirLight->setCastsShadow(true);
    dirLight->setShadowRange(1.5);
    scene->addLight(dirLight);

    {
        // Add a module to run the viewer
        imstkNew<VTKOpenVRViewer> viewer("VRViewer");
        viewer->setActiveScene(scene);

        // Add a module to run the scene
        imstkNew<SceneManager> sceneManager("Scene Manager");
        sceneManager->setActiveScene(scene);
        viewer->addChildThread(sceneManager);         // Start/stop scene with the view

        // Add a VR controller for the scalpel handle
        imstkNew<SceneObjectController> controller1(scalpelHandle, viewer->getVRDeviceClient(OPENVR_RIGHT_CONTROLLER));
        scene->addController(controller1);
        // Add a VR controller for the scalpel blade
        imstkNew<SceneObjectController> controller2(scalpelBlade10, viewer->getVRDeviceClient(OPENVR_RIGHT_CONTROLLER));
        scene->addController(controller2);

        // VRButtonPress emitted from viewer, recieve it in the scene that way the scene is not running whilst it is swapped
        bool blade10InHand = true;
        queueConnect<VRButtonEvent>(viewer->getVRDeviceClient(OPENVR_RIGHT_CONTROLLER), EventType::VRButtonPress, sceneManager,
            [&](VRButtonEvent* e)
        {
            // When any button pressed, swap blade
            // todo: distance metric not working
            if (e->m_buttonState == BUTTON_PRESSED)
            {
                const Vec3d& posControl = viewer->getVRDeviceClient(OPENVR_RIGHT_CONTROLLER)->getPosition();
                if (blade10InHand)
                {
                    // Swap to blade 15 only if it's close in space
                    Vec3d min, max;
                    scalpelBlade15->getMasterGeometry()->computeBoundingBox(min, max);
                    const Vec3d posBlade = (min + max) * 0.5;
                    const double dist    = (posControl - posBlade).norm();
                    LOG(INFO) << "Dist: " << dist;
                    if (dist < 2.0)
                    {
                        const Vec3d t = scalpelBlade15->getMasterGeometry()->getTranslation();
                        const Mat3d r = scalpelBlade15->getMasterGeometry()->getRotation();

                        // Set the new blade to move
                        controller2->setControlledSceneObject(scalpelBlade15);
                        blade10InHand = false;

                        scalpelBlade10->getMasterGeometry()->setTranslation(t);
                        scalpelBlade10->getMasterGeometry()->setRotation(r);
                    }
                }
                else
                {
                    // Swap to blade 10 only if it's close in space
                    Vec3d min, max;
                    scalpelBlade10->getMasterGeometry()->computeBoundingBox(min, max);
                    const Vec3d posBlade = (min + max) * 0.5;
                    const double dist    = (posControl - posBlade).norm();
                    LOG(INFO) << "Dist: " << dist;
                    if (dist < 2.0)
                    {
                        const Vec3d t = scalpelBlade10->getMasterGeometry()->getTranslation();
                        const Mat3d r = scalpelBlade10->getMasterGeometry()->getRotation();

                        controller2->setControlledSceneObject(scalpelBlade10);
                        blade10InHand = true;

                        // Swap transforms of the blades
                        scalpelBlade15->getMasterGeometry()->setTranslation(t);
                        scalpelBlade15->getMasterGeometry()->setRotation(r);
                    }
                }
            }
                        });

        // Start running
        viewer->requestStatus(ThreadStatus::Running);
        viewer->start();
    }

    return 0;
}