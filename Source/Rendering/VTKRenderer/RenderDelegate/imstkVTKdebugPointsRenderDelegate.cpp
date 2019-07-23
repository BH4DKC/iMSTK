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

#include "imstkVTKdebugPointsRenderDelegate.h"

#include "imstkSurfaceMesh.h"

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPoints.h>
#include <vtkTrivialProducer.h>
#include <vtkDoubleArray.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkOpenGLPolyDataMapper.h>

namespace imstk
{
VTKdbgPointsRenderDelegate::VTKdbgPointsRenderDelegate(const std::shared_ptr<DebugRenderPoints>& pointRenderData) :
    m_RenderGeoData(pointRenderData),
    m_mappedVertexArray(vtkSmartPointer<vtkDoubleArray>::New())
{
    // Map vertices
    m_mappedVertexArray->SetNumberOfComponents(3);

    // Create points
    m_points = vtkSmartPointer<vtkPoints>::New();
    m_points->SetData(m_mappedVertexArray);

    // Create PolyData
    m_polyData = vtkSmartPointer<vtkPolyData>::New();
    m_polyData->SetPoints(m_points);

    auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    m_glyph = vtkSmartPointer<vtkGlyph3D>::New();
    m_glyph->SetSourceConnection(sphereSource->GetOutputPort());
    m_glyph->SetInputData(m_polyData);

    // Update Transform, Render Properties
    updateActorProperties();
    setUpMapper(m_glyph->GetOutputPort(), false, m_RenderGeoData->getRenderMaterial());

    updateDataSource();
}

void
VTKdbgPointsRenderDelegate::updateDataSource()
{
    if (m_RenderGeoData->isModified())
    {
        m_RenderGeoData->setDataModified(false);
        m_mappedVertexArray->SetArray(m_RenderGeoData->getVertexBufferPtr(),
                                      m_RenderGeoData->getNumVertices() * 3, 1);

        // Update points geometry
        // m_Points need to be created from scrach, otherwise program will crash
        m_points = vtkSmartPointer<vtkPoints>::New();
        m_points->SetNumberOfPoints(m_RenderGeoData->getNumVertices());
        m_points->SetData(m_mappedVertexArray);
        m_polyData->SetPoints(m_points);

        m_mappedVertexArray->Modified();

        // Sleep for a while, wating for the data to propagate
        // This is necessary to avoid access violation error during CPU/GPU data transfer
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
} // imstk
