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

#pragma once

#include <memory>
#include <vector>

#include "imstkMath.h"
#include "imstkTextureManager.h"
#include "imstkVTKTextureDelegate.h"
#include "imstkRenderer.h"

#include "vtkSmartPointer.h"
#include "vtkRenderer.h"
#include "vtkProp.h"
#include "vtkCamera.h"
#include "vtkLight.h"

#ifdef iMSTK_ENABLE_VR
#include "vtkOpenVRRenderer.h"
#include "vtkOpenVRCamera.h"
#include "vtkOpenVRRenderWindow.h"
#include "vtkOpenVRRenderWindowInteractor.h"
#include "vtkInteractorStyle3D.h"
#include "vtkOpenVROverlayInternal.h"
#endif

class vtkAxesActor;

namespace imstk
{
class Scene;
class Camera;
class VTKRenderDelegate;

///
/// \class Renderer
///
/// \brief
///
class VTKRenderer : public Renderer
{
public:
    ///
    /// \brief Constructor
    ///
    VTKRenderer(std::shared_ptr<Scene> scene, const bool enableVR);

    ///
    /// \brief Default destructor
    ///
    ~VTKRenderer() = default;

    ///
    /// \brief Set/Get the rendering mode which defined the
    /// visibility of the renderer actors and the default camera
    ///
    void setMode(const Mode mode, const bool enableVR) override;

    ///
    /// \brief Change the debug axes length
    ///
    void setAxesLength(const double x, const double y, const double z);

    ///
    /// \brief Get the debug axes length
    ///
    double* getAxesLength();

    ///
    /// \brief Change the visibility of the debug axes
    ///
    void setAxesVisibility(const bool visible);

    ///
    /// \brief Returns whether the debug axes is visible or not
    ///
    bool getAxesVisibility() const;

    ///
    /// \brief Updates the scene camera's position and orientation
    ///
    void updateSceneCamera(std::shared_ptr<Camera> imstkCam);

    ///
    /// \brief Updates the render delegates
    ///
    void updateRenderDelegates();

    ///
    /// \brief Get the render delegates
    ///
    const std::vector<std::shared_ptr<VTKRenderDelegate>>& getDebugRenderDelegates() const { return m_debugRenderDelegates; }

    ///
    /// \brief Returns VTK renderer
    ///
    vtkSmartPointer<vtkRenderer> getVtkRenderer() const;

    ///
    /// \brief Update background colors
    ///
    void updateBackground(const Vec3d color1, const Vec3d color2 = Vec3d::Zero(), const bool gradientBackground = false);

protected:
    ///
    /// \brief Remove actors (also called props) from the scene
    ///
    void removeActors(const std::vector<vtkSmartPointer<vtkProp>>& actorList);

    ///
    /// \brief Add actors (also called props) from the scene
    ///
    void addActors(const std::vector<vtkSmartPointer<vtkProp>>& actorList);

    vtkSmartPointer<vtkRenderer>           m_vtkRenderer;
    vtkSmartPointer<vtkCamera>             m_defaultVtkCamera;
    vtkSmartPointer<vtkCamera>             m_sceneVtkCamera;
    std::vector<vtkSmartPointer<vtkLight>> m_vtkLights;
    std::vector<vtkSmartPointer<vtkProp>>  m_objectVtkActors;
    std::vector<vtkSmartPointer<vtkProp>>  m_debugVtkActors;
    vtkSmartPointer<vtkAxesActor>          m_AxesActor;

    std::vector<std::shared_ptr<VTKRenderDelegate>> m_renderDelegates;
    std::vector<std::shared_ptr<VTKRenderDelegate>> m_debugRenderDelegates;

    std::shared_ptr<Scene> m_scene;

    TextureManager<VTKTextureDelegate> m_textureManager;
#ifdef iMSTK_ENABLE_VR
    std::vector<vtkOpenVRCameraPose> m_camPos;
#endif
};
}
