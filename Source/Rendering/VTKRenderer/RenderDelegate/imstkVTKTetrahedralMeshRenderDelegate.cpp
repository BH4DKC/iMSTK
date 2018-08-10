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

#include "imstkVTKTetrahedralMeshRenderDelegate.h"

#include "imstkTetrahedralMesh.h"

#include <vtkUnstructuredGrid.h>
#include <vtkDataSetMapper.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkNew.h>
#include <vtkLookupTable.h>
#include <vtkColorSeries.h>

namespace imstk
{
VTKTetrahedralMeshRenderDelegate::VTKTetrahedralMeshRenderDelegate(std::shared_ptr<TetrahedralMesh> tetrahedralMesh) :
    m_geometry(tetrahedralMesh),
    m_mappedVertexArray(vtkSmartPointer<vtkDoubleArray>::New())
{
    // Map vertices
    StdVectorOfVec3d& vertices = m_geometry->getVertexPositionsNotConst();
    double* vertData = reinterpret_cast<double*>(vertices.data());
    m_mappedVertexArray->SetNumberOfComponents(3);
    m_mappedVertexArray->SetArray(vertData, vertices.size()*3, 1);

    // Create points
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(m_geometry->getNumVertices());
    points->SetData(m_mappedVertexArray);

    // Copy cells
    auto cells = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType cell[4];
    for(const auto &t : m_geometry->getTetrahedraVertices())
    {
        for(size_t i = 0; i < 4; ++i)
        {
            cell[i] = t[i];
        }
        cells->InsertNextCell(4,cell);
    }



    // Create Unstructured Grid
    m_meshConnectivity = vtkSmartPointer<vtkUnstructuredGrid>::New();
    m_meshConnectivity->SetPoints(points);
    m_meshConnectivity->SetCells(VTK_TETRA, cells);

    //---------------------------------------------------------------------------

    vtkSmartPointer<vtkFloatArray> scalars = vtkSmartPointer<vtkFloatArray>::New();
    if(m_geometry->renderScalarData())
    {
        // Create scalar data to associate with the vertices of the sphere
        auto scalarData = m_geometry->getNodalScalarData();
        auto numPts = scalarData.size();
        scalars->SetNumberOfValues(numPts);
        for (int i = 0; i < numPts; ++i)
        {
            scalars->SetValue(i, scalarData[i]);
        }
        m_meshConnectivity->GetPointData()->SetScalars(scalars);
        
    }
    
    //---------------------------------------------------------------------------

    // Mapper & Actor
    auto mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(m_meshConnectivity);

    if (m_geometry->renderScalarData())
    {
        vtkSmartPointer<vtkLookupTable> hueLut = vtkSmartPointer<vtkLookupTable>::New();        
        auto *ran = scalars->GetRange();
        hueLut->SetTableRange(ran[0], ran[1]);

        vtkSmartPointer<vtkColorSeries> colSer = vtkSmartPointer<vtkColorSeries>::New();
        colSer->SetColorScheme(vtkColorSeries::BREWER_SEQUENTIAL_BLUE_GREEN_3);
        colSer->BuildLookupTable(hueLut);

        hueLut->SetNumberOfTableValues(3);
        hueLut->SetTableValue(0, 0.95, 0.95, 0.95, 1.);
        hueLut->SetTableValue(1, 34. / 255., 139. / 255., 34. / 255., 1.);
        hueLut->SetTableValue(2, 1., 127./255., 80./255., 1.);


        //hueLut->Build();
        hueLut->SetAnnotation(vtkVariant{ 1 }, "1");
        hueLut->SetAnnotation(vtkVariant{ 3 }, "3");
        hueLut->SetAnnotation(vtkVariant{ 4 }, "4");

        mapper->SetLookupTable(hueLut);

        mapper->SetUseLookupTableScalarRange(1);
    }

    // Actor
    m_actor->SetMapper(mapper);

    // Update Transform, Render Properties
    this->update();
}

void
VTKTetrahedralMeshRenderDelegate::updateDataSource()
{
    if (m_geometry->m_dataModified)
    {
        m_mappedVertexArray->Modified();
        m_geometry->m_dataModified = false;
    }

    if (m_geometry->getTopologyChangedFlag())
    {
        auto& maskedTets = std::dynamic_pointer_cast<TetrahedralMesh>(m_geometry)->getRemovedTetrahedra();

        auto cells = m_meshConnectivity->GetCells();
        vtkIdType cell[4] = { 0, 0, 0, 0 };

        // Alter the masked cells to set the connectivity to {0, 0, 0, 0}
        for (size_t i = 0; i < m_geometry->getNumTetrahedra(); ++i)
        {
            if (maskedTets[i])
            {
                cells->ReplaceCell(5*i, 4, cell);
            }
        }
        m_meshConnectivity->Modified();
        m_geometry->setTopologyChangedFlag(false);

        /*auto scalars = vtkSmartPointer<vtkFloatArray>::New();
        auto scalarData = m_geometry->getNodalScalarData();
        for (size_t i = 0; i < m_geometry->getNumVertices(); ++i)
        {
            if (scalarData[i] != 4)
            {
                scalarData[i] = 1;
            }
        }
        scalars->SetArray(scalarData.data(), scalarData.size(), 1);
        m_meshConnectivity->GetPointData()->SetScalars(scalars);*/
    }
}


std::shared_ptr<Geometry>
VTKTetrahedralMeshRenderDelegate::getGeometry() const
{
    return m_geometry;
}
} // imstk
