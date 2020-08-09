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

#include "imstkTestingUtils.h"
#include "imstkRenderMaterial.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkRenderMaterialTest : public TestWithTempFolder
{
protected:
};

TEST_F(imstkRenderMaterialTest, SetGetTexture)
{
    {
        auto normalTexture =
            std::make_shared<Texture>("/test/path/to/texture", Texture::Type::Normal);
        auto anotherNormalTexture =
            std::make_shared<Texture>("/test/path/to/texture", Texture::Type::Normal);

        RenderMaterial material;
        material.addTexture(normalTexture);
        EXPECT_EQ(material.getTexture(Texture::Type::Normal), normalTexture);
        material.addTexture(anotherNormalTexture);
        EXPECT_EQ(material.getTexture(Texture::Type::Normal), anotherNormalTexture);
    }

    {
        auto metalTexture =
            std::make_shared<Texture>("/another/path/to/texture", Texture::Type::Metalness);
        auto samePathButDiffuseTexture =
            std::make_shared<Texture>("/another/path/to/texture", Texture::Type::Diffuse);

        RenderMaterial material;
        material.addTexture(metalTexture);
        EXPECT_EQ(material.getTexture(Texture::Type::Metalness), metalTexture);
        material.addTexture(samePathButDiffuseTexture);
        EXPECT_EQ(material.getTexture(Texture::Type::Diffuse), samePathButDiffuseTexture);
    }

    {
        auto noneTexture =
            std::make_shared<Texture>("/none", Texture::Type::None);

        RenderMaterial material;
        material.addTexture(noneTexture);
        EXPECT_EQ(material.getTexture(Texture::Type::None), nullptr);
    }

    {
        RenderMaterial material;
        EXPECT_EQ(material.getTexture(Texture::Type::Cubemap)->getPath(), "");
    }
}

TEST_F(imstkRenderMaterialTest, SetGetRoughness)
{
    RenderMaterial material;
    material.setRoughness(-1.0f);
    EXPECT_EQ(material.getRoughness(), 0.0f);

    material.setRoughness(0.0f);
    EXPECT_EQ(material.getRoughness(), 0.0f);

    material.setRoughness(100.0f);
    EXPECT_EQ(material.getRoughness(), 1.0f);

    material.setRoughness(1.0f);
    EXPECT_EQ(material.getRoughness(), 1.0f);

    material.setRoughness(0.6f);
    EXPECT_EQ(material.getRoughness(), 0.6f);
}

TEST_F(imstkRenderMaterialTest, SetGetMetalness)
{
    RenderMaterial material;
    material.setMetalness(-1.0f);
    EXPECT_EQ(material.getMetalness(), 0.0f);

    material.setMetalness(0.0f);
    EXPECT_EQ(material.getMetalness(), 0.0f);

    material.setMetalness(100.0f);
    EXPECT_EQ(material.getMetalness(), 1.0f);

    material.setMetalness(1.0f);
    EXPECT_EQ(material.getMetalness(), 1.0f);

    material.setMetalness(0.6f);
    EXPECT_EQ(material.getMetalness(), 0.6f);
}

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkRenderMaterialTest, Serialization)
{
    RenderMaterial material;
    auto texture = std::make_shared<Texture>("/test/path/to/texture", Texture::Type::Normal);
    auto otherTexture = std::make_shared<Texture>("/another/path/to/texture", Texture::Type::Metalness);
    auto lastTexture = std::make_shared<Texture>("/last/path/to/texture", Texture::Type::Diffuse);

    material.addTexture(texture);
    material.addTexture(otherTexture);
    material.addTexture(lastTexture);

    material.setDisplayMode(RenderMaterial::DisplayMode::WireframeSurface);
    material.setTessellated(true);
    material.setLineWidth(-0.10f);
    material.setLineWidth(3);
    material.setBackFaceCulling(false);
    material.setColor(Color::DarkGray);
    material.setMetalness(0.3f);
    material.setRoughness(0.2f);
    material.setEmissivity(5.19323329f);
    material.setReceivesShadows(false);
    material.setCastsShadows(false);
    material.setBlendMode(RenderMaterial::BlendMode::MaximumIntensity);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkRenderMaterial.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(material);
    }

    // Deserialize
    auto newMaterial = RenderMaterial();
        {
        std::ifstream is(getTempFolder() + "/imstkRenderMaterial.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newMaterial);
    }

    EXPECT_EQ(
        material.getTexture(Texture::Type::Normal)->getPath(),
        newMaterial.getTexture(Texture::Type::Normal)->getPath());
    EXPECT_EQ(
        material.getTexture(Texture::Type::Metalness)->getPath(),
        newMaterial.getTexture(Texture::Type::Metalness)->getPath());
    EXPECT_EQ(
        material.getTexture(Texture::Type::Diffuse)->getPath(),
        newMaterial.getTexture(Texture::Type::Diffuse)->getPath());
    EXPECT_EQ(
        material.getTexture(Texture::Type::Cavity)->getPath(),
        ""); // Test texture isn't added
    EXPECT_EQ(material.getDisplayMode(), newMaterial.getDisplayMode());
    EXPECT_EQ(material.getTessellated(), newMaterial.getTessellated());
    EXPECT_EQ(material.getLineWidth(), newMaterial.getLineWidth());
    EXPECT_EQ(material.getPointSize(), newMaterial.getPointSize());
    EXPECT_EQ(material.getBackFaceCulling(), newMaterial.getBackFaceCulling());
    EXPECT_EQ(material.getColor(), newMaterial.getColor());
    EXPECT_EQ(material.getMetalness(), newMaterial.getMetalness());
    EXPECT_EQ(material.getRoughness(), newMaterial.getRoughness());
    EXPECT_EQ(material.getEmissivity(), newMaterial.getEmissivity());
    EXPECT_EQ(material.getReceivesShadows(), newMaterial.getReceivesShadows());
    EXPECT_EQ(material.getCastsShadows(), newMaterial.getCastsShadows());
    EXPECT_EQ(material.getBlendMode(), newMaterial.getBlendMode());
}
#endif

int
imstkRenderMaterialTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
