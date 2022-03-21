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

#include <memory>
#include <string>
#include <unordered_map>

namespace imstk
{
class Geometry;
class RenderMaterial;
class Renderer;

///
/// \class VisualModel
///
/// \brief Contains geometric, material, and render information
///
class VisualModel : public EventObject
{
public:
    VisualModel();
    ~VisualModel() override = default;

    // *INDENT-OFF*
    SIGNAL(VisualModel, modified);
    // *INDENT-ON*

    ///
    /// \brief Get/Set geometry
    ///@{
    std::shared_ptr<Geometry> getGeometry() const { return m_geometry; }
    void setGeometry(std::shared_ptr<Geometry> geometry) { m_geometry = geometry; }
    ///@}

    ///
    /// \brief Get/Set name
    ///@{
    const std::string& getName() const { return m_name; }
    void setName(std::string name) { m_name = name; }
    ///@}

    ///
    /// \brief Get/Set the delegate hint, which helps indicate
    /// how to render this VisualModel
    ///@{
    const std::string getDelegateHint() const;
    void setDelegateHint(const std::string& name) { m_delegateHint = name; }
    ///@}

    ///
    /// \brief Set/Get render material
    ///@{
    void setRenderMaterial(std::shared_ptr<RenderMaterial> renderMaterial);
    std::shared_ptr<RenderMaterial> getRenderMaterial() const { return m_renderMaterial; }
    ///@}

    ///
    /// \brief Visibility functions
    ///@{
    void show() { setIsVisible(true); }
    void hide() { setIsVisible(false); }
    bool isVisible() const { return m_isVisible; }
    void setIsVisible(const bool visible);
    ///@}

    ///
    /// \brief Get/Set whether the delegate has been created
    ///@{
    bool getRenderDelegateCreated(Renderer* ren);
    void setRenderDelegateCreated(Renderer* ren, bool created) { m_renderDelegateCreated[ren] = created; }
    ///@}

    void postModified() { this->postEvent(Event(VisualModel::modified())); }

protected:
    std::string m_name;
    std::string m_delegateHint;

    std::shared_ptr<Geometry>       m_geometry;
    std::shared_ptr<RenderMaterial> m_renderMaterial;

    bool m_isVisible; ///< true if mesh is shown, false if mesh is hidden
    std::unordered_map<Renderer*, bool> m_renderDelegateCreated;
};
} // namespace imstk
