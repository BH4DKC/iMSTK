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

#include "imstkVolumetricMesh.h"

#include "g3log/g3log.hpp"

namespace imstk
{
std::shared_ptr<SurfaceMesh>
VolumetricMesh::getAttachedSurfaceMesh()
{
    if (m_attachedSurfaceMesh == nullptr)
    {
        LOG(WARNING) << "VolumetricMesh::getAttachedSurfaceMesh warning: attachedSurfaceMesh not set.\n"
                     << "Extract a surface mesh to attach using computeAttachedSurfaceMesh().";
    }
    return m_attachedSurfaceMesh;
}

void
VolumetricMesh::setAttachedSurfaceMesh(std::shared_ptr<SurfaceMesh> surfaceMesh)
{
    m_attachedSurfaceMesh = surfaceMesh;
}
} // imstk