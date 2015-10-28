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
//---------------------------------------------------------------------------
//
// Authors:
//
// Contact:
//---------------------------------------------------------------------------

#include "Core/Model.h"
#include "Core/Geometry.h"
#include "Core/RenderDelegate.h"
#include "Core/Factory.h"
#include "Mesh/SurfaceMesh.h"

class MeshRenderDelegate : public RenderDelegate
{
public:
    virtual void draw() const override;
    virtual bool isTargetTextured() const override;
};

void MeshRenderDelegate::draw() const
{
    auto geom = this->getSourceGeometryAs<SurfaceMesh>();
    if(!geom)
    {
        return;
    }

    auto mesh = std::static_pointer_cast<SurfaceMesh>(geom->shared_from_this());
    OpenGLRenderer::drawSurfaceMeshTriangles(mesh, geom->getRenderDetail());

    if(geom->getRenderDetail()->renderType & SIMMEDTK_RENDER_NORMALS)
    {
        OpenGLRenderer::drawNormals(mesh,
                                    geom->getRenderDetail()->normalColor,
                                    geom->getRenderDetail()->normalLength);
    }
}

bool MeshRenderDelegate::isTargetTextured() const
{
    auto geom = this->getSourceGeometryAs<SurfaceMesh>();
    if(!geom)
    {
        return false;
    }
    return geom->isMeshTextured();
}

RegisterFactoryClass(RenderDelegate,
                     MeshRenderDelegate,
                     RenderDelegate::RendererType::Other)