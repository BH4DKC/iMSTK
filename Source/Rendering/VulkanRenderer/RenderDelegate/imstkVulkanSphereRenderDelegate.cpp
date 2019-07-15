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

#include "imstkVulkanSphereRenderDelegate.h"

namespace imstk
{
VulkanSphereRenderDelegate::VulkanSphereRenderDelegate(std::shared_ptr<VisualModel> visualModel,
                                                       SceneObject::Type            type,
                                                       VulkanMemoryManager&         memoryManager)
{
    m_visualModel = visualModel;

    auto geometry = std::static_pointer_cast<Sphere>(visualModel->getGeometry());

    auto radius = geometry->getRadius();

    auto source = vtkSmartPointer<vtkSphereSource>::New();
    source->SetCenter(WORLD_ORIGIN[0], WORLD_ORIGIN[1], WORLD_ORIGIN[2]);
    source->SetPhiResolution(20);
    source->SetThetaResolution(20);
    source->SetRadius(radius);
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

        VulkanBasicVertex sphereVertex;

        sphereVertex.position[0] = position[0];
        sphereVertex.position[1] = position[1];
        sphereVertex.position[2] = position[2];

        sphereVertex.normal[0] = normal[0];
        sphereVertex.normal[1] = normal[1];
        sphereVertex.normal[2] = normal[2];

        m_sphereVertices.push_back(sphereVertex);
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

        m_sphereTriangles.push_back(trianglePoints);
    }

    m_numVertices  = (uint32_t)m_sphereVertices.size();
    m_numTriangles = (uint32_t)m_sphereTriangles.size();
    m_vertexSize   = sizeof(VulkanBasicVertex);

    if (!m_visualModel->getRenderMaterial())
    {
        m_visualModel->setRenderMaterial(std::make_shared<RenderMaterial>());
    }

    this->initializeData(memoryManager, this->getVisualModel()->getRenderMaterial());

    m_vertexBuffer->updateVertexBuffer(&m_sphereVertices, &m_sphereTriangles);

    this->update(0);
}

void
VulkanSphereRenderDelegate::update(const uint32_t frameIndex)
{
    this->updateUniforms(frameIndex);
}
}