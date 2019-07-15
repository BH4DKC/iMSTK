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

#include "imstkVulkanPlaneRenderDelegate.h"

namespace imstk
{
VulkanPlaneRenderDelegate::VulkanPlaneRenderDelegate(std::shared_ptr<VisualModel> visualModel,
                                                     SceneObject::Type            type,
                                                     VulkanMemoryManager&         memoryManager)
{
    this->initialize(visualModel);
    auto geometry = std::static_pointer_cast<Plane>(visualModel->getGeometry());

    auto width = geometry->getWidth();

    auto source = vtkSmartPointer<vtkPlaneSource>::New();
    source->SetCenter(WORLD_ORIGIN[0], WORLD_ORIGIN[1], WORLD_ORIGIN[2]);
    source->SetNormal(UP_VECTOR[0], UP_VECTOR[1], UP_VECTOR[2]);
    source->Update();

    auto triangulate = vtkSmartPointer<vtkTriangleFilter>::New();
    triangulate->SetInputConnection(source->GetOutputPort());

    triangulate->Update();

    auto sourceData = triangulate->GetOutput();

    auto positions = sourceData->GetPoints();
    auto normals   = sourceData->GetPointData()->GetNormals();
    auto triangles = sourceData->GetPolys();

    for (int i = 0; i < sourceData->GetNumberOfPoints(); i++)
    {
        auto position = positions->GetPoint(i);
        auto normal   = normals->GetTuple(i);

        VulkanBasicVertex planeVertex;

        planeVertex.position[0] = position[0] * width;
        planeVertex.position[1] = position[1] * width;
        planeVertex.position[2] = position[2] * width;

        planeVertex.normal[0] = normal[0];
        planeVertex.normal[1] = normal[1];
        planeVertex.normal[2] = normal[2];

        m_planeVertices.push_back(planeVertex);
    }

    triangles->InitTraversal();

    for (int i = 0; i < triangles->GetNumberOfCells(); i++)
    {
        auto points   = vtkSmartPointer<vtkIdList>::New();
        auto triangle = triangles->GetNextCell(points);

        std::array<uint32_t, 3> trianglePoints = {
            (uint32_t)points->GetId(0),
            (uint32_t)points->GetId(1),
            (uint32_t)points->GetId(2)
        };

        m_planeTriangles.push_back(trianglePoints);
    }

    m_numVertices  = (uint32_t)m_planeVertices.size();
    m_numTriangles = (uint32_t)m_planeTriangles.size();
    m_vertexSize   = sizeof(VulkanBasicVertex);

    this->initializeData(memoryManager, this->getVisualModel()->getRenderMaterial());

    m_vertexBuffer->updateVertexBuffer(&m_planeVertices, &m_planeTriangles);

    for (uint32_t i = 0; i < memoryManager.m_buffering; i++)
    {
        this->update(i);
    }
}

void
VulkanPlaneRenderDelegate::update(const uint32_t frameIndex)
{
    this->updateUniforms(frameIndex);
}
}