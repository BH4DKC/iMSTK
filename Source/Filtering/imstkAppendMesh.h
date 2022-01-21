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

#include "imstkGeometryAlgorithm.h"

namespace imstk
{
class SurfaceMesh;

///
/// \class AppendMesh
///
/// \brief This filter appends two SurfaceMeshes, no topological connections are made
///
class AppendMesh : public GeometryAlgorithm
{
public:
    AppendMesh();
    ~AppendMesh() override = default;

    void addInputMesh(std::shared_ptr<SurfaceMesh> inputMesh);
    std::shared_ptr<SurfaceMesh> getOutputMesh() const;

protected:
    void requestUpdate() override;
};
} // namespace imstk