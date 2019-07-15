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

#include "vulkan/vulkan.h"
#include "glm/glm.hpp"

#include "imstkVulkanMaterialDelegate.h"
#include "imstkVulkanVertexBuffer.h"
#include "imstkVulkanFramebuffer.h"
#include "imstkVulkanTextureDelegate.h"

#include <memory>
#include <vector>

namespace imstk
{
class VulkanRenderer;

class VulkanPostProcess
{
public:
    ///
    /// \brief Constructor
    ///
    VulkanPostProcess(VulkanRenderer* renderer, uint32_t numViews, unsigned int level = 0);
    VulkanPostProcess(VulkanRenderer* renderer, uint32_t numViews, unsigned int width, unsigned int height);

    void addInputImage(
        VkSampler*    sampler,
        VkImageView*  imageView,
        VkImageLayout layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    void generateMipmaps(VkCommandBuffer& commandBuffer,
                         unsigned int     levels,
                         VkImage&         image);
protected:
    friend class VulkanRenderer;
    friend class VulkanPostProcessingChain;

    ///
    /// \brief Creates a parent pipeline object that gets inherited by other materials
    ///
    void createPipeline(VulkanRenderer* renderer, std::string fragmentSource);
    void createRenderPass(VulkanRenderer* renderer);
    void createFramebuffer(VulkanRenderer*    renderer,
                           const unsigned int width,
                           const unsigned int height);

    void createFullscreenQuad(VulkanRenderer* renderer);

    virtual void initialize(VulkanRenderer* renderer,
                            std::string = VulkanShaderPath::PostProcessing + "postprocess_frag.spv");
    void initializeFramebuffer(VulkanRenderer* renderer);

    ///
    /// \brief Updates image information to current layout after renderpass
    ///
    void updateImageLayouts();

    ///
    /// \brief Set image attachments to a readable layout
    ///
    void setAttachmentsToReadLayout(VkCommandBuffer* commandBuffer,
                                    uint32_t         queueFamily,
                                    const uint32_t   numViews);

    void createDescriptors(VulkanRenderer* renderer);
    void createDescriptorSetLayouts(VulkanRenderer* renderer);
    void createDescriptorPool(VulkanRenderer* renderer);
    void createDescriptorSets(VulkanRenderer* renderer);

    void clear(VkDevice* device);

    VkPipeline m_pipeline;
    VkGraphicsPipelineCreateInfo m_graphicsPipelineInfo;
    VkPipelineLayout m_pipelineLayout;
    VulkanMaterialPipelineComponents m_pipelineComponents;

    VkDescriptorPool m_descriptorPool;
    std::vector<VkDescriptorSet>       m_descriptorSets;
    std::vector<VkDescriptorSetLayout> m_descriptorSetLayouts;
    std::vector<VkWriteDescriptorSet>  m_writeDescriptorSets;

    std::shared_ptr<VulkanVertexBuffer> m_vertexBuffer;

    std::shared_ptr<VulkanFramebuffer> m_framebuffer;

    // Resources
    std::vector<VkSampler*>    m_samplers;
    std::vector<VkImageView*>  m_imageViews;
    std::vector<VkImageLayout> m_layouts;

    unsigned int m_downsampleLevels = 0;
    unsigned int m_outputIndex      = 0;
    uint32_t     m_numViews;

    std::vector<VkAttachmentReference> m_colorAttachments;
    VkRenderPass m_renderPass;
    float        m_pushConstantData[32];
};
}
