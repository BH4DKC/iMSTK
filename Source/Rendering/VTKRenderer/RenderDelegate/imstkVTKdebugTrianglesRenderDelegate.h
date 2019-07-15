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

#include "imstkVTKRenderDelegate.h"

class vtkDoubleArray;

namespace imstk
{
class SurfaceMesh;

///
/// \class SurfaceMeshRenderDelegate
///
/// \brief
///
class VTKdbgTrianglesRenderDelegate : public VTKRenderDelegate
{
public:
    ///
    /// \brief Constructor
    ///
    VTKdbgTrianglesRenderDelegate(const std::shared_ptr<DebugRenderTriangles>& renderTriangles);

    ///
    /// \brief Update polydata source based on the surface mesh geometry
    ///
    void updateDataSource() override;

protected:
    std::shared_ptr<DebugRenderTriangles> m_RenderGeoData; ///> Geometry to render

    // Auxiliary variables for rendering
    vtkSmartPointer<vtkDoubleArray> m_pappedVertexArray;
    vtkSmartPointer<vtkPoints>      m_points;
    vtkSmartPointer<vtkCellArray>   m_cellArray;
    vtkSmartPointer<vtkPolyData>    m_polyData;
};
}
