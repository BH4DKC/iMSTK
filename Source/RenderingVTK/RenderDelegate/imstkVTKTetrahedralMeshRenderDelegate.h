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

#include "imstkVTKPolyDataRenderDelegate.h"

class vtkDataArray;
class vtkDoubleArray;
class vtkUnstructuredGrid;
class vtkCellArray;

namespace imstk
{
class TetrahedralMesh;
template<typename T, int N> class VecDataArray;

///
/// \class VTKTetrahedralMeshRenderDelegate
///
/// \brief Tetrahedral mesh render delegate with vtk render backend
///
class VTKTetrahedralMeshRenderDelegate : public VTKPolyDataRenderDelegate
{
public:
    VTKTetrahedralMeshRenderDelegate(std::shared_ptr<VisualModel> visualModel);
    ~VTKTetrahedralMeshRenderDelegate() override = default;

    ///
    /// \brief Process handling of messages recieved
    ///
    void processEvents() override;

protected:
    ///
    /// \brief Callback for when vertex values are modified
    ///
    void vertexDataModified(Event* e);
    void indexDataModified(Event* e);

    ///
    /// \brief Callback for when geometry is modified
    ///
    void geometryModified(Event* e);

    void setVertexBuffer(std::shared_ptr<VecDataArray<double, 3>> vertices);
    void setIndexBuffer(std::shared_ptr<VecDataArray<int, 4>> indices);

    std::shared_ptr<VecDataArray<double, 3>> m_vertices;
    std::shared_ptr<VecDataArray<int, 4>>    m_indices;

    vtkSmartPointer<vtkUnstructuredGrid> m_mesh;               ///> Mapped tetrahedral mesh

    vtkSmartPointer<vtkDoubleArray> m_mappedVertexArray;       ///> Mapped array of vertices
    vtkSmartPointer<vtkDataArray>   m_mappedVertexScalarArray; ///> Mapped array of scalars
    vtkSmartPointer<vtkCellArray>   m_cellArray;               ///> Array of cells
};
} // namespace imstk