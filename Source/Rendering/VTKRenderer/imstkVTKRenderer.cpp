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

#include "imstkVTKRenderer.h"

#include "imstkScene.h"
#include "imstkCamera.h"
#include "imstkVTKRenderDelegate.h"
#include "imstkVTKSurfaceMeshRenderDelegate.h"

#include "vtkLightActor.h"
#include "vtkCameraActor.h"
#include "vtkAxesActor.h"
#include "vtkCullerCollection.h"

#include "g3log/g3log.hpp"

namespace imstk
{
VTKRenderer::VTKRenderer(std::shared_ptr<Scene> scene)
{
    m_scene = scene;
    this->updateRenderDelegates();

    // Initialize textures for surface mesh render delegates
    for ( const auto& renderDelegate : m_renderDelegates )
    {
        auto smRenderDelegate = std::dynamic_pointer_cast<VTKSurfaceMeshRenderDelegate>(renderDelegate);
        if (smRenderDelegate)
        {
            smRenderDelegate->initializeTextures(m_textureManager);
        }
    }

    // Lights and light actors
    for ( const auto& light : scene->getLights() )
    {
        m_vtkLights.push_back( light->getVtkLight() );
        if (light->getType() == LightType::POINT_LIGHT ||
            light->getType() == LightType::SPOT_LIGHT)
        {
            auto lightActor = vtkSmartPointer<vtkLightActor>::New();
            lightActor->SetLight( light->getVtkLight() );
            m_debugVtkActors.push_back( lightActor );
        }
    }

    // Global Axis
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);
    axes->SetAxisLabels(false);
    //m_debugVtkActors.push_back( axes );

    // Camera and camera actor
    m_sceneVtkCamera = vtkSmartPointer<vtkCamera>::New();
    this->updateSceneCamera(scene->getCamera());
    auto camActor = vtkSmartPointer<vtkCameraActor>::New();
    camActor->SetCamera(  m_sceneVtkCamera );
    //m_debugVtkActors.push_back( camActor );

    // Debug camera
    m_defaultVtkCamera = m_vtkRenderer->GetActiveCamera();

    ///TODO : based on scene properties
    // Customize background colors
    m_vtkRenderer->SetBackground(0.66,0.66,0.66);
    m_vtkRenderer->SetBackground2(0.406, 0.481, 0.497);
    m_vtkRenderer->GradientBackgroundOn();

    ///TODO : only on deformable objects with UseBounds (need vtk update)
    // Remove culling
    if (auto culler = m_vtkRenderer->GetCullers()->GetLastItem())
    {
        m_vtkRenderer->RemoveCuller(culler);
    }

    this->setMode(Mode::SIMULATION);
}

vtkSmartPointer<vtkRenderer>
VTKRenderer::getVtkRenderer() const
{
    return m_vtkRenderer;
}

void
VTKRenderer::setMode(Renderer::Mode mode)
{
    if( mode == Mode::EMPTY && m_currentMode != Mode::EMPTY )
    {
        this->removeActors(m_objectVtkActors);
        m_vtkRenderer->RemoveAllLights();

        if( m_currentMode == Mode::DEBUG )
        {
            this->removeActors(m_debugVtkActors);
        }

        m_vtkRenderer->SetActiveCamera(m_defaultVtkCamera);
    }
    else if( mode == Mode::DEBUG && m_currentMode != Mode::DEBUG )
    {
        this->addActors(m_debugVtkActors);

        if( m_currentMode == Mode::EMPTY )
        {
            this->addActors(m_objectVtkActors);
            for ( const auto& light : m_vtkLights )
            {
                m_vtkRenderer->AddLight(light);
            }
        }

        m_vtkRenderer->SetActiveCamera(m_defaultVtkCamera);
        m_vtkRenderer->ResetCamera();
    }
    else if ( mode == Mode::SIMULATION && m_currentMode != Mode::SIMULATION )
    {
        if( m_currentMode == Mode::EMPTY )
        {
            this->addActors(m_objectVtkActors);
            for ( const auto& light : m_vtkLights )
            {
                m_vtkRenderer->AddLight(light);
            }
        }
        else if( m_currentMode == Mode::DEBUG )
        {
            this->removeActors(m_debugVtkActors);
        }

        m_vtkRenderer->SetActiveCamera(m_sceneVtkCamera);
        m_vtkRenderer->ResetCameraClippingRange();
    }

    m_currentMode = mode;
}

void
VTKRenderer::updateSceneCamera(std::shared_ptr<Camera> imstkCam)
{
    // Get imstk Camera info
    auto p = imstkCam->getPosition();
    auto f = imstkCam->getFocalPoint();
    auto v = imstkCam->getViewUp();

    // Update vtk Camera
    m_sceneVtkCamera->SetPosition(p[0], p[1], p[2]);
    m_sceneVtkCamera->SetFocalPoint(f[0], f[1], f[2]);
    m_sceneVtkCamera->SetViewUp(v[0], v[1], v[2]);
    m_sceneVtkCamera->SetViewAngle(imstkCam->getViewAngle());
}

void
VTKRenderer::updateRenderDelegates()
{
    // Object actors
    for ( const auto& obj : m_scene->getSceneObjects() )
    {
        auto geom = obj->getVisualGeometry();
        if (geom && !geom->m_renderDelegateCreated)
        {
            auto delegate = VTKRenderDelegate::make_delegate( geom );
            if (delegate == nullptr)
            {
                LOG(WARNING) << "Renderer::Renderer error: Could not create render delegate for '"
                             << obj->getName() << "'.";
                continue;
            }

            m_renderDelegates.push_back( delegate );
            m_objectVtkActors.push_back( delegate->getVtkActor() );
            m_vtkRenderer->AddActor(delegate->getVtkActor());
            geom->m_renderDelegateCreated = true;
        }
    }

    for (auto delegate : m_renderDelegates)
    {
        delegate->update();
    }
}

void
VTKRenderer::removeActors(const std::vector<vtkSmartPointer<vtkProp>>& actorList)
{
    for ( const auto& actor : actorList )
    {
        m_vtkRenderer->RemoveActor(actor);
    }
}

void
VTKRenderer::addActors(const std::vector<vtkSmartPointer<vtkProp>>& actorList)
{
    for ( const auto& actor : actorList )
    {
        m_vtkRenderer->AddActor(actor);
    }
}

void
VTKRenderer::updateBackground(const Vec3d backgroundOne, const Vec3d backgroundTwo /*= Vec3d::Zero()*/, const bool gradientBackground /*= false*/)
{
    m_vtkRenderer->SetBackground(backgroundOne.x(), backgroundOne.y(), backgroundOne.z());
    if (gradientBackground)
    {
        m_vtkRenderer->SetBackground2(backgroundTwo.x(), backgroundTwo.y(), backgroundTwo.z());
        m_vtkRenderer->GradientBackgroundOn();
    }
    else
    {
        m_vtkRenderer->GradientBackgroundOff();
    }
}
} // imstk
