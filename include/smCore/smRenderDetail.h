// This file is part of the SimMedTK project.
// Copyright (c) Center for Modeling, Simulation, and Imaging in Medicine,
//                        Rensselaer Polytechnic Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#ifndef SMRENDERDETAIL_H
#define SMRENDERDETAIL_H

#include "smCore/smConfig.h"
#include "smCore/smColor.h"

#include <memory>
#include <vector>

class smVisualArtifact;
class smCoreClass;
struct smUnifiedId;

///\brief Hold a pointer to a source of geometry that render details can use for drawing.
struct smGeometrySource {
  smCoreClass* sceneObject;
  smVisualArtifact* analyticObject;

  smGeometrySource()
    : sceneObject(nullptr), analyticObject(nullptr)
    { }
  void setSource(smCoreClass* src)
    {
    this->sceneObject = src;
    this->analyticObject = nullptr;
    }
  void setSource(smVisualArtifact* src)
    {
    this->sceneObject = nullptr;
    this->analyticObject = src;
    }
  template<typename T>
  T* sourceAs() const
    {
    T* result;
    if ((result = dynamic_cast<T*>(sceneObject)))
      return result;
    if ((result = dynamic_cast<T*>(analyticObject)))
      return result;
    return nullptr;
    }
};

/// \brief smRenderDetail has rendering options and features.
///It shows how the mesh should be rendered
struct smRenderDetail
{
public:
    typedef std::shared_ptr<smRenderDetail> Ptr;

    smRenderDetail();
    smRenderDetail(smUInt type);

    void reset();

    /// \brief attachment of shader
    void addShader(std::shared_ptr<smUnifiedId> p_shaderID);

    /// \brief attachment of VAO
    void addVAO(std::shared_ptr<smUnifiedId> p_shaderID);

    const smColor &getColorDiffuse() const;

    const smColor &getColorAmbient() const;

    const smColor &getColorSpecular() const;

    const smFloat &getShininess() const;

    const smUInt &getRenderType() const;

    const smFloat &getPointSize() const;

    const smFloat &getLineSize() const;

    const smColor &getNormalColor() const;

    const smColor &getHighLightColor() const;

    const smColor &getVertexColor() const;

    const smColor &getShadowColor() const;

    const smBool &getCastShadow() const;

    const smBool &getCanGetShadow() const;

    const smColor &getWireFrameColor() const;

    const smBool &getDebugDraw() const;

    const std::vector<std::shared_ptr<smUnifiedId>> &getShaders() const;

    const std::vector<smBool> &getShaderEnable() const;

    const std::vector<std::shared_ptr<smUnifiedId>> &getVAOs() const;

    const std::vector<smBool> &getVAOEnable() const;

    void setPointSize(const float size);

    void setLineSize(const float size);

    void setVertexColor(const smColor vertColor);

    void setHighlightColor(const smColor highlightColor);

    void setNormalColor(const smColor highlightColor);

    void setShininess(const smFloat s);

    void setNormalLength(const smFloat len);

    void setDiffuseColor(const smColor diffColor);

    void setAmbientColor(const smColor ambColor);

    void setSpecularColor(const smColor specColor);

    void setShadowColor(const smColor shadColor);

    void setWireframeColor(const smColor wireColor);

public:
    smUInt renderType; // render type
    smColor colorDiffuse; // diffuse color
    smColor colorAmbient; // ambient color
    smColor colorSpecular; // specular color
    smColor highLightColor; // highlight color
    smColor vertexRenderColor; // vertex color
    smColor shadowColor; // shadow color
    smBool castShadow; // object can generate a shadow or not
    smBool canGetShadow; // object can get the shadow or not
    smColor normalColor; // normal color
    smColor wireFrameColor; // wire frame color
    smFloat pointSize; // point size if rendering of vertices are enabled
    smFloat lineSize; // line width size
    smFloat shininess; // specular shinness
    smBool debugDraw; // debug draw enabled or not
    smFloat normalLength; // length of rendered normals
    std::vector<std::shared_ptr<smUnifiedId>> shaders; // attached shaders
    std::vector<smBool> shaderEnable; // enable/disable any attached shader
    std::vector<std::shared_ptr<smUnifiedId>> VAOs; // stores  VAO IDs
    std::vector<smBool> VAOEnable; // enable/disable any attached VAO
};

#endif // SMRENDERDETAIL_H