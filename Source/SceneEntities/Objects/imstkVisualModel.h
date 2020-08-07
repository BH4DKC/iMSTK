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

namespace imstk
{
class Geometry;
class RenderMaterial;
class DebugRenderGeometry;
///
/// \class VisualModel
///
/// \brief Contains geometric, material, and render information
///
class VisualModel
{
public:
    ///
    /// \brief Constructor
    ///
    explicit VisualModel(std::shared_ptr<Geometry> geometry);
    explicit VisualModel(std::shared_ptr<Geometry>       geometry,
                         std::shared_ptr<RenderMaterial> renderMaterial);
    explicit VisualModel(std::shared_ptr<DebugRenderGeometry> geometry);
    explicit VisualModel(std::shared_ptr<DebugRenderGeometry> geometry,
                         std::shared_ptr<RenderMaterial>      renderMaterial);

    VisualModel() = delete;

    ///
    /// \brief Get/set geometry
    ///
    std::shared_ptr<Geometry> getGeometry();
    void setGeometry(std::shared_ptr<Geometry> geometry);

    ///
    /// \brief Get/set geometry
    ///
    std::shared_ptr<DebugRenderGeometry> getDebugGeometry();
    void setDebugGeometry(std::shared_ptr<DebugRenderGeometry> geometry);

    ///
    /// \brief Set/Get render material
    ///
    void setRenderMaterial(std::shared_ptr<RenderMaterial> renderMaterial);
    std::shared_ptr<RenderMaterial> getRenderMaterial() const;

    ///
    /// \brief Visibility functions
    ///
    void show();
    void hide();
    bool isVisible() const;

    ///
    /// \brief Return true if renderer delegate is created
    ///
    bool isRenderDelegateCreated();

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive)
    {
        archive(
            iMSTK_SERIALIZE(geometry),
            iMSTK_SERIALIZE(renderMaterial),
            iMSTK_SERIALIZE(isVisible),
            iMSTK_SERIALIZE(renderDelegateCreated)
        );
    }
#endif

protected:
    friend class VulkanRenderDelegate;
    friend class VTKRenderer;

    std::shared_ptr<Geometry> m_geometry = nullptr;
    std::shared_ptr<DebugRenderGeometry> m_DbgGeometry = nullptr;
    std::shared_ptr<RenderMaterial>      m_renderMaterial;

    bool m_isVisible = true;              ///< true if mesh is shown, false if mesh is hidden
    bool m_renderDelegateCreated = false; ///< true if RenderDelegate has been created
};
}
