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

#include "imstkColor.h"
#include "imstkSerialize.h"
#include "imstkTexture.h"
#include "imstkTextureManager.h"

#include <vector>

namespace imstk
{
class ColorFunction;
class Texture;

///
/// \class RenderMaterial
/// \brief TODO
///
class RenderMaterial
{
public:
    /// Display mode for the scene objects
    enum class DisplayMode
    {
        Surface,
        Wireframe,
        Points,
        WireframeSurface,
        VolumeRendering,
        Fluid,               ///< Renders a set of points using a screen-space fluid renderer
        Image
    };

    /// surface shading model. Defaults to Phong
    enum class ShadingModel
    {
        Phong,   ///< Phong shading model (default)
        Gouraud, ///< Gouraud shading model (default)
        Flat,    ///< Flat shading model with no interpolation
        PBR      ///< Physically based rendering
    };

    /// Volume rendering blend mode
    enum class BlendMode
    {
        Alpha,
        Additive,
        MaximumIntensity,
        MinimumIntensity
    };

    ///
    /// \brief Constructor
    ///
    RenderMaterial();

    ///
    /// \brief Get/Set display mode
    ///
    DisplayMode getDisplayMode() const;
    void setDisplayMode(const DisplayMode displayMode);

    ///
    /// \brief Get/Set tessellated
    ///
    bool getTessellated() const;
    void setTessellated(const bool tessellated);

    ///
    /// \brief Get/Set line width or the wireframe
    ///
    float getLineWidth() const;
    void setLineWidth(const float width);

    ///
    /// \brief Get/Set point radius
    ///
    float getPointSize() const;
    ///
    /// \brief Get/Set point radius
    ///
    void setPointSize(const float size);

    ///
    /// \brief Backface culling on/off
    ///
    bool getBackFaceCulling() const;
    void setBackFaceCulling(const bool culling);
    void backfaceCullingOn();
    void backfaceCullingOff();

    ///
    /// \brief Get/Set the color. This affects the diffuse color directly, but
    /// it affects the specular color in the case of metals.
    ///
    const Color& getDiffuseColor() const;
    void setDiffuseColor(const Color& color);
    const Color& getColor() const;
    void setColor(const Color& color);

    ///
    /// \brief Get/Set the specular color
    ///
    const Color& getSpecularColor() const;
    void setSpecularColor(const Color& color);

    ///
    /// \brief Get/Set the ambient color
    ///
    const Color& getAmbientColor() const;
    void setAmbientColor(const Color& color);

    ///
    /// \brief Get/Set the metalness
    ///
    const float& getMetalness() const;
    void setMetalness(const float metalness);

    ///
    /// \brief Get/Set ambient light coefficient
    ///
    const float& getAmbientLightCoeff() const { return m_ambientLightCoeff; };
    void setAmbientLightCoeff(const float a) { m_ambientLightCoeff = a; };

    ///
    /// \brief Get/Set ambient light coefficient
    ///
    const float& getSpecularPower() const { return m_specularPower; };
    void setSpecularPower(const float p) { m_specularPower = p; };

    ///
    /// \brief Get/Set the roughness
    ///
    const float& getRoughness() const;
    void setRoughness(const float roughness);

    ///
    /// \brief Get/Set emissivity
    ///
    const float& getEmissivity() const;
    void setEmissivity(const float emissivity);

    ///
    /// \brief Add/Get texture
    ///
    void addTexture(std::shared_ptr<Texture> texture);
    std::shared_ptr<Texture> getTexture(Texture::Type type);

    ///
    /// \brief Get/Set shadow receiving ability
    ///
    void setReceivesShadows(const bool receivesShadows);
    bool getReceivesShadows() const;

    ///
    /// \brief Get/Set shadow cast status
    ///
    void setCastsShadows(const bool castsShadows);
    bool getCastsShadows() const;

    ///
    /// \brief Get/Set edge visibility
    ///
    void setEdgeVisibility(const bool visibility) { m_edgeVisibility = visibility; };
    bool getEdgeVisibility() const { return m_edgeVisibility; };

    ///
    /// \brief Get/Set blend mode
    /// This function only works for volumes, particles and decals currently
    /// and the MAXIMUM_INTENSITY and MINIMUM_INTENSITY blend modes are only available for volumes
    ///
    virtual void setBlendMode(const BlendMode blendMode);
    const BlendMode getBlendMode();

    ///
    /// \brief Checks if the material must be handled uniquely
    ///
    bool isDecal();
    bool isParticle();
    bool isLineMesh();

    DisplayMode getRenderMode() const { return m_displayMode; };
    ShadingModel getShadingModel() const { return m_shadingModel; };
    void setShadingModel(const ShadingModel& model) { m_shadingModel = model; }

    bool isModified() const { return m_modified; };
    void setModified(const bool modified) { m_modified = modified; };

    float getOcclusionStrength() const { return m_occlusionStrength; }
    void setOcclusionStrength(const float o) { m_occlusionStrength = o; };

    float getNormalStrength() const { return m_normalStrength; }
    void setNormalnStrength(const float n) { m_normalStrength = n; };

    const Color& getEdgeColor() const { return m_edgeColor; };
    void setEdgeColor(const Color& color) { m_edgeColor = color; };

    const Color& getVertexColor() const { return m_vertexColor; };
    void setVertexColor(const Color& color) { m_vertexColor = color; };

    double getOpacity() const { return m_opacity; }
    void setOpacity(const float opacity) { m_opacity = opacity; };

    bool getBackfaceCulling() const { return m_backfaceCulling; };
    void setBackfaceCulling(const bool c) { m_backfaceCulling = c; };

    std::shared_ptr<ColorFunction> getColorLookupTable() const { return m_lookupTable; }
    void setColorLookupTable(std::shared_ptr<ColorFunction> lut) { this->m_lookupTable = lut; }

    bool getScalarVisibility() const { return m_scalarVisibility; }
    void setScalarVisibility(bool scalarVisibility) { this->m_scalarVisibility = scalarVisibility; }

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive)
    {
        archive(
            iMSTK_SERIALIZE(textures),
            iMSTK_SERIALIZE(blendMode),
            iMSTK_SERIALIZE(diffuseColor),
            iMSTK_SERIALIZE(specularColor),
            iMSTK_SERIALIZE(ambientColor),
            iMSTK_SERIALIZE(ambientLightCoeff),
            iMSTK_SERIALIZE(specularPower),
            iMSTK_SERIALIZE(opacity),
            iMSTK_SERIALIZE(lineWidth),
            iMSTK_SERIALIZE(pointSize),
            iMSTK_SERIALIZE(edgeColor),
            iMSTK_SERIALIZE(vertexColor),
            iMSTK_SERIALIZE(edgeVisibility),
            iMSTK_SERIALIZE(vertexVisibility),
            iMSTK_SERIALIZE(emissivity),
            iMSTK_SERIALIZE(emissivityColor),
            iMSTK_SERIALIZE(metalness),
            iMSTK_SERIALIZE(roughness),
            iMSTK_SERIALIZE(occlusionStrength),
            iMSTK_SERIALIZE(normalStrength),
            iMSTK_SERIALIZE(imageBasedLighting),
            iMSTK_SERIALIZE(receivesShadows),
            iMSTK_SERIALIZE(castsShadows),
            iMSTK_SERIALIZE(stateModified),
            iMSTK_SERIALIZE(modified),
            iMSTK_SERIALIZE(backfaceCulling),
            iMSTK_SERIALIZE(shadingModel),
            iMSTK_SERIALIZE(backfaceCulling),
            iMSTK_SERIALIZE(tessellated),
            iMSTK_SERIALIZE(isDecal),
            iMSTK_SERIALIZE(isLineMesh),
            iMSTK_SERIALIZE(isParticle),
            iMSTK_SERIALIZE(scalarVisibility)
        );
    }
#endif

protected:
    friend class VTKRenderDelegate;
    friend class VulkanRenderDelegate;
    friend class VulkanDecalRenderDelegate;
    friend class VulkanLineMeshRenderDelegate;
    friend class VulkanParticleRenderDelegate;
    friend class VTKdbgLinesRenderDelegate;

    // Textures
    std::vector<std::shared_ptr<Texture>> m_textures; ///< Ordered by Texture::Type

    ///--------------Volume rendering properties----------------
    BlendMode m_blendMode = BlendMode::Alpha;

    ///-------------------Common properties---------------------
    Color m_diffuseColor  = Color::LightGray;
    Color m_specularColor = Color::Red;
    Color m_ambientColor  = Color::White;

    float m_ambientLightCoeff = 0.1f;
    float m_specularPower     = 100.f;
    float m_opacity = 1.f;

    ///-------------Wireframe specific properties----------------
    float m_lineWidth        = 1.f;
    float m_pointSize        = 2.f;
    Color m_edgeColor        = Color::Marigold;
    Color m_vertexColor      = Color::Teal;
    bool  m_edgeVisibility   = true; ///< \note not used (vtk backend)
    bool  m_vertexVisibility = true; ///< \note not used (vtk backend)

    ///----------------PBR specific properties-------------------
    float m_emissivity    = 0.f;
    Color m_emmisiveColor = Color::White;

    float m_metalness = 0.f;  ///< Value for metalness with range: [0.0, 1.0]
    float m_roughness = 10.f; ///< Value for roughness with range: [0.0, 1.0]

    float m_occlusionStrength = 10.f;
    float m_normalStrength    = 1.f;

    ///---------------------Global states------------------------
    bool m_imageBasedLighting = false;

    // Shadows
    bool m_receivesShadows = true; ///< \note not implemented
    bool m_castsShadows    = true; ///< \note not implemented

    /// \todo remove one of these?
    bool m_stateModified   = true;      ///< Flag for expensive state changes
    bool m_modified        = true;      ///< Flag for any material property changes
    bool m_backfaceCulling = true;      ///< For performance, uncommon for this to be false

    DisplayMode  m_displayMode  = DisplayMode::Surface;
    ShadingModel m_shadingModel = ShadingModel::Phong;

    bool m_tessellated = false;
    bool m_isDecal     = false;
    bool m_isLineMesh  = false;
    bool m_isParticle  = false;

    std::shared_ptr<ColorFunction> m_lookupTable;
    bool m_scalarVisibility = false;
};
}
