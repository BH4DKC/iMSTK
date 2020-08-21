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

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "vtksys/SystemTools.hxx"

#include "imstkTestingUtils.h"
#include "imstkLight.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkLightTest : public TestWithTempFolder
{
protected:
    DirectionalLight m_directional;
    PointLight m_point;
    SpotLight m_spot;
};

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkLightTest, DirectionalLightSerialization)
{
    m_directional.setName("Serialization_DirectionalLight");
    m_directional.setFocalPoint(1.0, 2.0, static_cast<float>(PI));
    m_directional.setColor(Color::Red);
    m_directional.setIntensity(300.0);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkDirectionalLight.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(m_directional);
    }

    // Deserialize
    auto newLight = DirectionalLight("newLight");
    {
        std::ifstream is(getTempFolder() + "/imstkDirectionalLight.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newLight);
    }

    EXPECT_EQ(m_directional.getName(), newLight.getName());
    EXPECT_EQ(m_directional.getFocalPoint(), newLight.getFocalPoint());
    EXPECT_EQ(m_directional.getColor(), newLight.getColor());
    EXPECT_EQ(m_directional.getIntensity(), newLight.getIntensity());
}

TEST_F(imstkLightTest, DirectionalLightSerialization_SharedPointer)
{
    std::shared_ptr<DirectionalLight> light = std::make_shared<DirectionalLight>("DirectionalLight_SharedPtr");

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkDirectionalLight.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(light);
    }

    // Deserialize
    auto newLight = std::make_shared<DirectionalLight>();
    {
        std::ifstream is(getTempFolder() + "/imstkDirectionalLight.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newLight);
    }

    EXPECT_EQ(light->getName(), newLight->getName());
    EXPECT_EQ(light->getFocalPoint(), newLight->getFocalPoint());
    EXPECT_EQ(light->getColor(), newLight->getColor());
    EXPECT_EQ(light->getIntensity(), newLight->getIntensity());
}

TEST_F(imstkLightTest, PointLightSerialization)
{
    m_point.setName("Serialization_PointLight");
    m_point.setConeAngle(3.0);
    m_point.setPosition(1.0, 3.0, -1.0);

    // Serialize
    {
        std::ofstream os("imstkPointLight.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

       archive(m_point);
    }

    // Deserialize
    auto newLight = PointLight("newLight");
    {
        std::ifstream is("imstkPointLight.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newLight);
    }

    EXPECT_EQ(m_point.getName(), newLight.getName());
    EXPECT_EQ(m_point.getConeAngle(), newLight.getConeAngle());
    EXPECT_EQ(m_point.getPosition(), newLight.getPosition());
}


TEST_F(imstkLightTest, SpotLightSerialization)
{
    m_spot.setName("Serialization_SpotLight");
    m_spot.setSpotAngle(PI);

    // Serialize
    {
        std::ofstream os("imstkSpotLight.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(m_spot);
    }

    // Deserialize
    auto newLight = SpotLight("newLight");
    {
        std::ifstream is("imstkSpotLight.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newLight);
    }

    EXPECT_EQ(m_spot.getName(), newLight.getName());
    EXPECT_EQ(m_spot.getSpotAngle(), newLight.getSpotAngle());
}

#endif

int
imstkLightTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
