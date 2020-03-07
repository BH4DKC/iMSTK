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

#include "imstkViewer.h"

namespace imstk
{
const std::shared_ptr<Scene>&
Viewer::getActiveScene() const
{
    return m_activeScene;
}

const std::shared_ptr<Renderer>&
Viewer::getActiveRenderer() const
{
    if (m_activeScene)
    {
        return m_rendererMap.at(m_activeScene);
    }
    else
    {
        return nullptr;
    }
}

const bool&
Viewer::isRendering() const
{
    return m_running;
}

const std::shared_ptr<ScreenCaptureUtility>&
Viewer::getScreenCaptureUtility() const
{
    return m_screenCapturer;
}

const std::shared_ptr<GUIOverlay::Canvas>&
Viewer::getCanvas()
{
    return m_canvas;
}

void
Viewer::setOnCharFunction(char c, EventHandlerFunction func)
{
    m_interactorStyle->m_onCharFunctionMap[c] = func;
}

void
Viewer::setOnMouseMoveFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onMouseMoveFunction = func;
}

void
Viewer::setOnLeftButtonDownFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onLeftButtonDownFunction = func;
}

void
Viewer::setOnLeftButtonUpFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onLeftButtonUpFunction = func;
}

void
Viewer::setOnMiddleButtonDownFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onMiddleButtonDownFunction = func;
}

void
Viewer::setOnMiddleButtonUpFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onMiddleButtonUpFunction = func;
}

void
Viewer::setOnRightButtonDownFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onRightButtonDownFunction = func;
}

void
Viewer::setOnRightButtonUpFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onRightButtonUpFunction = func;
}

void
Viewer::setOnMouseWheelForwardFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onMouseWheelForwardFunction = func;
}

void
Viewer::setOnMouseWheelBackwardFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onMouseWheelBackwardFunction = func;
}

void
Viewer::setOnTimerFunction(EventHandlerFunction func)
{
    m_interactorStyle->m_onTimerFunction = func;
}
}
