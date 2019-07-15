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

#include "imstkVulkanDecalRenderDelegate.h"

namespace imstk
{
VulkanDecalRenderDelegate::VulkanDecalRenderDelegate(std::shared_ptr<VisualModel> visualModel,
                                                     SceneObject::Type            type,
                                                     VulkanMemoryManager&         memoryManager)
{
    this->initialize(visualModel);

    auto geometry = std::static_pointer_cast<DecalPool>(visualModel->getGeometry());

    m_numVertices  = 8;
    m_numTriangles = 12;
    m_vertexSize   = sizeof(VulkanBasicVertex);

    m_visualModel->getRenderMaterial()->m_isDecal = true;

    this->initializeData(memoryManager, this->getVisualModel()->getRenderMaterial());

    this->updateVertexBuffer();
}

void
VulkanDecalRenderDelegate::updateVertexBuffer()
{
    auto vertices = (VulkanBasicVertex*)m_vertexBuffer->getVertexMemory();
    auto geometry = std::static_pointer_cast<DecalPool>(m_visualModel->getGeometry());

    for (unsigned i = 0; i < m_numVertices; i++)
    {
        vertices[i].position =
            geometry->m_vertexPositions[i];
    }

    auto triangles = (std::array<uint32_t, 3>*)m_vertexBuffer->getIndexMemory();

    for (unsigned i = 0; i < m_numTriangles; i++)
    {
        triangles[i][0] = (uint32_t)geometry->m_triangles[i].x;
        triangles[i][1] = (uint32_t)geometry->m_triangles[i].y;
        triangles[i][2] = (uint32_t)geometry->m_triangles[i].z;
    }
}

void
VulkanDecalRenderDelegate::initializeData(VulkanMemoryManager& memoryManager, std::shared_ptr<RenderMaterial> material)
{
    m_vertexUniformBuffer   = std::make_shared<VulkanUniformBuffer>(memoryManager, (uint32_t)sizeof(VulkanLocalDecalVertexUniforms));
    m_fragmentUniformBuffer = std::make_shared<VulkanUniformBuffer>(memoryManager, (uint32_t)sizeof(VulkanLocalDecalFragmentUniforms));

    m_material = std::make_shared<VulkanMaterialDelegate>(m_vertexUniformBuffer,
        m_fragmentUniformBuffer,
        material,
        memoryManager);

    m_vertexBuffer = std::make_shared<VulkanVertexBuffer>(memoryManager, m_numVertices, m_vertexSize, m_numTriangles);
}

void
VulkanDecalRenderDelegate::update(const uint32_t frameIndex, std::shared_ptr<Camera> camera)
{
    unsigned int index = 0;

    auto eye        = glm::tvec3<float>(camera->getPosition().x(), camera->getPosition().y(), camera->getPosition().z());
    auto center     = glm::tvec3<float>(camera->getFocalPoint().x(), camera->getFocalPoint().y(), camera->getFocalPoint().z());
    auto up         = glm::tvec3<float>(camera->getViewUp().x(), camera->getViewUp().y(), camera->getViewUp().z());
    auto viewMatrix = glm::lookAt(eye, center, up);

    auto geometry = std::static_pointer_cast<DecalPool>(m_visualModel->getGeometry());

    for (auto decal : geometry->getDecals())
    {
        decal->updateDecal(viewMatrix);
        m_decalVertexUniforms.transforms[index] = decal->m_transform;
        m_decalFragmentUniforms.inverses[index] = decal->m_inverse;
        index++;
    }

    auto mat = this->getVisualModel()->getRenderMaterial();

    auto color = mat->getColor();
    m_decalFragmentUniforms.color           = glm::vec4(color.r, color.g, color.b, color.a);
    m_decalFragmentUniforms.receivesShadows = mat->getReceivesShadows() ? 1 : 0;
    m_decalFragmentUniforms.emissivity      = mat->getEmissivity();
    m_decalFragmentUniforms.roughness       = mat->getRoughness();
    m_decalFragmentUniforms.metalness       = mat->getMetalness();

    m_vertexUniformBuffer->updateUniforms(sizeof(VulkanLocalDecalVertexUniforms),
        (void*)&m_decalVertexUniforms, frameIndex);
    m_fragmentUniformBuffer->updateUniforms(sizeof(VulkanLocalDecalFragmentUniforms),
        (void*)&m_decalFragmentUniforms, frameIndex);
}
}
