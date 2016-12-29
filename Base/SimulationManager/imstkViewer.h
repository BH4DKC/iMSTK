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

#ifndef imstkViewer_h
#define imstkViewer_h

#include <memory>
#include <unordered_map>

#include "imstkScene.h"
#include "imstkRenderer.h"
#include "imstkInteractorStyle.h"

#include "vtkSmartPointer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"

// 2D actor
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkProperty.h>

// Screenshot
#include "imstkScreenCaptureUtility.h"

namespace imstk
{

class SimulationManager;

///
/// \class Viewer
///
/// \brief Viewer
///
class Viewer
{
public:
    ///
    /// \brief
    ///
    Viewer(SimulationManager* manager = nullptr)
    {
        m_interactorStyle->setSimulationManager(manager);
        m_vtkRenderWindow->SetInteractor(m_vtkRenderWindow->MakeRenderWindowInteractor());
        m_vtkRenderWindow->GetInteractor()->SetInteractorStyle( m_interactorStyle );
        m_vtkRenderWindow->SetSize(1000,800);

        //2d actor
        m_textActor = vtkSmartPointer<vtkTextActor>::New();
    }

    ///
    /// \brief
    ///
    ~Viewer() = default;

    ///
    /// \brief
    ///
    std::shared_ptr<Scene> getCurrentScene() const;

    ///
    /// \brief
    ///
    void setCurrentScene(std::shared_ptr<Scene>scene);

    ///
    /// \brief
    ///
    std::shared_ptr<Renderer> getCurrentRenderer() const;

    ///
    /// \brief
    ///
    void setRenderingMode(Renderer::Mode mode);

    ///
    /// \brief
    ///
    void startRenderingLoop();

    ///
    /// \brief
    ///
    void endRenderingLoop();

    ///
    /// \brief
    ///
    vtkSmartPointer<vtkRenderWindow>getVtkRenderWindow() const;

    ///
    /// \brief
    ///
    const bool& isRendering() const;

    ///
    /// \brief
    ///
    void setScreenCaptureState(const bool state)
    {
        m_triggerScreencapture = state;
    }

    ///
    /// \brief
    ///
    bool shouldICaptureScreen() const
    {
        return m_triggerScreencapture;
    }

    ///
    /// \brief Enable the screen capture
    ///
    void enableScreenCapture(const std::string prefix = "screenShot-")
    {
        if (m_screenCapturer == nullptr)
        {
            m_screenCapturer = std::make_shared<screenCaptureUtility>(prefix);
        }
        m_screencaptureEnabled = true;
    }

    ///
    /// \brief
    ///
    void disableScreenCapture(const std::string prefix = "screenShot-")
    {
        m_screencaptureEnabled = false;
    }

    ///
    /// \brief
    ///
    void captureScreen() const
    {
        m_screenCapturer->saveScreenShot(this->getVtkRenderWindow());
    }

    ///
    /// \brief
    ///
    bool isScreencaptureEnabled() const
    {
        return m_screencaptureEnabled;
    }

    ///
    /// \brief
    ///
    vtkSmartPointer<vtkTextActor> get2DActor()
    {
        return m_textActor;
    }

    void setTextTo2DActor(std::string& text, Vec3d color= Vec3d::Ones())
    {
        m_textActor->SetInput(text.c_str());
        m_textActor->GetTextProperty()->SetColor(color.x(), color.y(), color.z());
    }

    ///
    /// \brief Set custom event handler for keypress event
    ///
    void setCustomOnCharEventHandler(vtkSlotFunctionType func, const char c)
    {
        m_interactorStyle->setOnCharEventHandler(func, c);
    }

protected:

    vtkSmartPointer<vtkRenderWindow> m_vtkRenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    vtkSmartPointer<InteractorStyle> m_interactorStyle = vtkSmartPointer<InteractorStyle>::New();
    std::shared_ptr<Scene> m_currentScene;
    std::unordered_map<std::shared_ptr<Scene>, std::shared_ptr<Renderer>> m_rendererMap;
    bool m_running = false;

    // Screen capture
    bool m_triggerScreencapture = false;
    bool m_screencaptureEnabled = false;
    std::shared_ptr<screenCaptureUtility> m_screenCapturer;

    // overlay 2d Actor
    vtkSmartPointer<vtkTextActor> m_textActor;
};

} // imstk

#endif // ifndef imstkViewer_h
