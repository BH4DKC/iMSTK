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

#include "imstkPointSet.h"
#include "imstkSurfaceMesh.h"

// Vega
#include "volumetricMesh.h"

namespace imstk
{
///
/// \class VolumetricMesh
///
/// \brief Base class for all volume mesh types
///
class VolumetricMesh : public PointSet
{
public:

    ///
    /// \brief Computes the attached surface mesh
    ///
    virtual void computeAttachedSurfaceMesh() = 0;

    ///
    /// \brief Returns the attached surface mesh
    ///
    std::shared_ptr<SurfaceMesh> getAttachedSurfaceMesh();

    ///
    /// \brief Sets the surface mesh that is attached
    ///
    void setAttachedSurfaceMesh(std::shared_ptr<SurfaceMesh> surfaceMesh);

protected:

    VolumetricMesh(Geometry::Type type, const std::string name = std::string("")) : PointSet(type, name) {}

    std::shared_ptr<SurfaceMesh> m_attachedSurfaceMesh;          ///> Attached surface mesh
};
} // imstk
