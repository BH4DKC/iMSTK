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

// imstk
#include "imstkVTKInteractorStyle.h"
#include "imstkSimulationManager.h"
#include "imstkVTKTextStatusManager.h"

// vtk
#include "vtkObjectFactory.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkCamera.h"
#include "vtkTextActor.h"

namespace imstk
{
VTKInteractorStyle::VTKInteractorStyle()
{
    m_textStatusManager = std::make_shared<VTKTextStatusManager>(this);
    m_lastFpsUpdate     = std::chrono::high_resolution_clock::now();
    m_lastFps           = 60.0;
}

VTKInteractorStyle::~VTKInteractorStyle()
{
    this->SetReferenceCount(0);
}

void
VTKInteractorStyle::SetCurrentRenderer(vtkRenderer* ren)
{
    // Remove actor if previous renderer
    if (this->CurrentRenderer)
    {
        for (int i = 0; i < VTKTextStatusManager::NumStatusTypes; ++i)
        {
            this->CurrentRenderer->RemoveActor2D(m_textStatusManager->getTextActor(i));
        }
    }

    // Set new current renderer
    vtkBaseInteractorStyle::SetCurrentRenderer(ren);

    // Add actor to current renderer
    for (int i = 0; i < VTKTextStatusManager::NumStatusTypes; ++i)
    {
        this->CurrentRenderer->AddActor2D(m_textStatusManager->getTextActor(i));
    }
}

void
VTKInteractorStyle::OnTimer()
{
    auto renderer = std::static_pointer_cast<VTKRenderer>(m_simManager->getViewer()->getActiveRenderer());

    // Update Camera
    renderer->updateSceneCamera(m_simManager->getActiveScene()->getCamera());

    // Update render delegates
    renderer->updateRenderDelegates();

    // Reset camera clipping range
    this->CurrentRenderer->ResetCameraClippingRange();

    if (m_displayFps)
    {
        // Update framerate value display
        auto   now       = std::chrono::high_resolution_clock::now();
        double visualFPS = 1e6 / static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now - m_pre).count());
        visualFPS = 0.1 * visualFPS + 0.9 * m_lastFps;
        m_lastFps = visualFPS;

        int t = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastFpsUpdate).count());
        if (t > 250) //wait 250ms before updating displayed value
        {
            double physicalFPS;
            if (m_simManager->getStatus() != SimulationStatus::paused)
            {
                physicalFPS = m_simManager->getActiveScene()->getFPS();
            }
            else
            {
                physicalFPS = -1.0; // negative value means paused
            }

            m_textStatusManager->setFPS(visualFPS, physicalFPS);
            m_lastFpsUpdate = now;
        }
        m_pre = now;
    }

    // Render
    this->Interactor->Render();
    m_post = std::chrono::high_resolution_clock::now();

    // Plan next render
    this->Interactor->CreateOneShotTimer(0);

    // Call custom behavior
    if (m_onTimerFunction)
    {
        // Call the custom behavior to run on every frame
        m_onTimerFunction(this);
    }
}

void
VTKInteractorStyle::OnChar()
{
    char key = this->Interactor->GetKeyCode();

    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onCharFunctionMap.count(key)
        && m_onCharFunctionMap.at(key)
        && m_onCharFunctionMap.at(key)(this))
    {
        return;
    }

    SimulationStatus status = m_simManager->getStatus();

    if (key == ' ')
    {
        // pause simulation
        if (status == SimulationStatus::running)
        {
            m_simManager->pause();
        }
        // play simulation
        else if (status == SimulationStatus::paused)
        {
            m_simManager->run();
        }
        // Launch simulation if inactive
        if (status == SimulationStatus::inactive)
        {
            m_textStatusManager->setStatusVisibility(VTKTextStatusManager::FPS, m_displayFps);
            m_simManager->start(SimulationStatus::running);
        }
    }
    else if (status != SimulationStatus::inactive
             && (key == 'q' || key == 'Q' || key == 'e' || key == 'E')) // end Simulation
    {
        m_textStatusManager->setStatusVisibility(VTKTextStatusManager::FPS, false);
        //m_simManager->endSimulation();
    }
    else if (key == 'd' || key == 'D')  // switch rendering mode
    {
        if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::SIMULATION)
        {
            m_simManager->getViewer()->setRenderingMode(Renderer::Mode::SIMULATION);
        }
        else
        {
            m_simManager->getViewer()->setRenderingMode(Renderer::Mode::DEBUG);
        }
    }
    else if (key == '\u001B')  // quit viewer
    {
        m_simManager->getViewer()->endRenderingLoop();
    }
    else if (key == 'p' || key == 'P')  // switch framerate display
    {
        m_displayFps = !m_displayFps;
        m_textStatusManager->setStatusVisibility(VTKTextStatusManager::FPS, m_displayFps);
        this->Interactor->Render();
    }
    else if (key == 'r' || key == 'R')
    {
        m_simManager->reset();
    }
}

void
VTKInteractorStyle::OnMouseMove()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onMouseMoveFunction
        && m_onMouseMoveFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnMouseMove();
}

void
VTKInteractorStyle::OnLeftButtonDown()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onLeftButtonDownFunction
        && m_onLeftButtonDownFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnLeftButtonDown();
}

void
VTKInteractorStyle::OnLeftButtonUp()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onLeftButtonUpFunction
        && m_onLeftButtonUpFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnLeftButtonUp();
}

void
VTKInteractorStyle::OnMiddleButtonDown()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onMiddleButtonDownFunction
        && m_onMiddleButtonDownFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnMiddleButtonDown();
}

void
VTKInteractorStyle::OnMiddleButtonUp()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onMiddleButtonUpFunction
        && m_onMiddleButtonUpFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnMiddleButtonUp();
}

void
VTKInteractorStyle::OnRightButtonDown()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onRightButtonDownFunction
        && m_onRightButtonDownFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnRightButtonDown();
}

void
VTKInteractorStyle::OnRightButtonUp()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onRightButtonUpFunction
        && m_onRightButtonUpFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnRightButtonUp();
}

void
VTKInteractorStyle::OnMouseWheelForward()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onMouseWheelForwardFunction
        && m_onMouseWheelForwardFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnMouseWheelForward();
}

void
VTKInteractorStyle::OnMouseWheelBackward()
{
    // Call custom function if exists, and return
    // if it returned `override=true`
    if (m_onMouseWheelBackwardFunction
        && m_onMouseWheelBackwardFunction(this))
    {
        return;
    }

    // Default behavior : ignore mouse if simulation active
    if (m_simManager->getViewer()->getRenderingMode() != Renderer::Mode::DEBUG)
    {
        return;
    }

    // Else : use base class interaction
    vtkBaseInteractorStyle::OnMouseWheelBackward();
}
} // imstk
