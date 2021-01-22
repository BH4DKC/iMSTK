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

#include "imstkEventObject.h"
#include "imstkMath.h"
#include "imstkColor.h"

namespace imstk
{

struct SSAOConfig
{
    bool m_enableSSAO = false;
    bool m_SSAOBlur = false; // blur occlusion
    double m_SSAORadius = 0.1; // comparison radius
    double m_SSAOBias = 0.001; // comparison bias
    unsigned int m_KernelSize = 128; // number of samples used
};

struct ShadowConfig
{
    bool m_enableShadows = false;
    unsigned int m_shadowResolution = 2048; // Shadow map resolution, not need to be power of 2
};

struct RendererConfig
{
    bool m_enableVR = false;

    Color m_BGColor1 = Color(0.3285, 0.3285, 0.6525);
    Color m_BGColor2 = Color(0.13836, 0.13836, 0.2748);

    // ScreenSpace Ambient Occlusion
    SSAOConfig m_ssaoConfig;

    // Shadow Settings
    ShadowConfig m_shadowConfig;
};

///
/// \class Renderer
///
/// \brief Rendering window manager and contains user API to configure the rendering with various backends
///
class Renderer : public EventObject
{
public:
    ///
    /// \brief Enumerations for the render mode
    ///
    enum class Mode
    {
        Empty,
        Debug,
        Simulation
    };

    Renderer() : m_config(std::make_shared<RendererConfig>()) { }
    virtual ~Renderer() = default;

public:
    ///
    /// \brief Set rendering mode
    ///
    virtual void setMode(const Renderer::Mode mode, const bool enableVR)
    {
        m_VrEnabled   = enableVR;
        m_currentMode = mode;
    }

    ///
    /// \brief Get rendering mode
    ///
    virtual const Renderer::Mode& getMode() const { return m_currentMode; }

    ///
    /// \brief Update background colors
    ///
    virtual void updateBackground(const Vec3d color1, const Vec3d color2 = Vec3d::Zero(), const bool gradientBackground = false) = 0;

    ///
    /// \brief Get the render config
    /// 
    std::shared_ptr<RendererConfig> getRenderConfig() const { return m_config; }

    ///
    /// \brief Apply config changes
    ///
    void updateConfig() { applyConfigChanges(m_config); }
    virtual void applyConfigChanges(std::shared_ptr<RendererConfig> config) = 0;

protected:
    bool m_VrEnabled = false;
    Renderer::Mode m_currentMode = Renderer::Mode::Simulation;

    std::shared_ptr<RendererConfig> m_config;
};
}
