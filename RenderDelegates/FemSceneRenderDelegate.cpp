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
#include "Simulators/FemSceneObject.h"
#include "Core/RenderDelegate.h"
#include "Core/Factory.h"
#include "Mesh/VolumeMesh.h"

class FemSceneRenderDelegate : public RenderDelegate
{
public:
  virtual void draw() const override;
};

void FemSceneRenderDelegate::draw() const
{
  auto v_mesh = this->getSourceGeometryAs<FemSceneObject>()->v_mesh;
  RenderDelegate::Ptr delegate = v_mesh->getRenderDelegate();
  if (delegate)
    delegate->draw();
}

RegisterFactoryClass(RenderDelegate,
                     FemSceneRenderDelegate,
                     RenderDelegate::RendererType::Other)