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
#include "imstkAPIUtilities.h"
#include "imstkVisualObjectImporter.h"

using namespace imstk;

///
/// \brief This example demonstrates configuring the renderer
/// objects, lights etc.
///
int
main()
{
    // simManager and Scene
    auto simManager = std::make_shared<SimulationManager>();
    auto scene      = simManager->createNewScene("Rendering");

    // Add IBL Probe
    auto globalIBLProbe = std::make_shared<IBLProbe>(
        iMSTK_DATA_ROOT "/IBL/roomIrradiance.dds",
        iMSTK_DATA_ROOT "/IBL/roomRadiance.dds",
        iMSTK_DATA_ROOT "/IBL/roomBRDF.png");
    scene->setGlobalIBLProbe(globalIBLProbe);

    // Head mesh
#ifdef iMSTK_USE_Vulkan
    auto headObject = VisualObjectImporter::importVisualObject(
        "head", iMSTK_DATA_ROOT "/head/head_revised.obj",
        iMSTK_DATA_ROOT "/head/", 1, Vec3d(0, 0, 0), "dds");
#else
    auto headObject = VisualObjectImporter::importVisualObject(
        "head", iMSTK_DATA_ROOT "/head/head_revised.obj",
        iMSTK_DATA_ROOT "/head/");

    // Head material
    auto headNormalTexture = std::make_shared<Texture>(iMSTK_DATA_ROOT "/head/HeadTexture_Normal.png", Texture::Type::NORMAL);
    auto headMaterial      = headObject->getVisualModel(0)->getRenderMaterial();
    headMaterial->addTexture(headNormalTexture);
#endif

    scene->addSceneObject(headObject);

    // Position camera
    auto cam = scene->getCamera();
    cam->setPosition(0, 0.25, 1);
    cam->setFocalPoint(0, 0.25, 0);

    // Lights
    auto directionalLight = std::make_shared<DirectionalLight>("DirectionalLight");
    directionalLight->setIntensity(7);
    directionalLight->setColor(Color(1.0, 0.95, 0.8));
    directionalLight->setCastsShadow(true);
    directionalLight->setShadowRange(1.5);
    scene->addLight(directionalLight);

    auto pointLight = std::make_shared<PointLight>("PointLight");
    pointLight->setIntensity(0.1);
    pointLight->setPosition(0.1, 0.2, 0.5);
    scene->addLight(pointLight);

    // Sphere
    auto sphereObj      = apiutils::createVisualAnalyticalSceneObject(Geometry::Type::Sphere, scene, "VisualSphere", 0.025);
    auto sphereMaterial = std::make_shared<RenderMaterial>();
    auto sphereMesh     = sphereObj->getVisualGeometry();
    sphereMesh->translate(0.1, 0.2, 0.5);
    sphereMaterial->setEmissivity(10);
    sphereMaterial->setCastsShadows(false);
    sphereObj->getVisualModel(0)->setRenderMaterial(sphereMaterial);

    // Plane
    auto planeObj      = apiutils::createVisualAnalyticalSceneObject(Geometry::Type::Plane, scene, "VisualPlane", 10);
    auto planeMaterial = std::make_shared<RenderMaterial>();
    planeMaterial->setColor(Color::DarkGray);
    planeObj->getVisualModel(0)->setRenderMaterial(planeMaterial);

    // Run
    simManager->setActiveScene(scene);
    simManager->getViewer()->setBackgroundColors(Vec3d(0, 0, 0));

#ifdef iMSTK_USE_Vulkan
    auto viewer = std::dynamic_pointer_cast<VulkanViewer>(simManager->getViewer());
    viewer->setResolution(1000, 800);
    viewer->disableVSync();
    //viewer->enableFullscreen();
#endif

    simManager->start(SimulationStatus::paused);

    return 0;
}
