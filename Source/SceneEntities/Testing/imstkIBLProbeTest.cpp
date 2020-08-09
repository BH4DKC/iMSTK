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
#include "imstkIBLProbe.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkIBLProbeTest : public TestWithTempFolder
{
protected:
};

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkIBLProbeTest, Serialization)
{
    IBLProbe probe(
        std::string("/irradiance/cube/map/path"),
        std::string("/radiance/cube/map/path"),
        std::string("/brdf/L/U/T/path")
        );

    // Serialize
    {
        std::ofstream os("imstkIBLProbeTest.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(probe);
    }

    // Deserialize
    auto newProbe = IBLProbe("/wrong", "/not/it", "/even/worse");
    {
        std::ifstream is("imstkIBLProbeTest.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newProbe);
    }

    EXPECT_EQ(probe.getIrradianceCubemapTexture()->getPath(), newProbe.getIrradianceCubemapTexture()->getPath());
    EXPECT_EQ(probe.getRadianceCubemapTexture()->getPath(), newProbe.getRadianceCubemapTexture()->getPath());
    EXPECT_EQ(probe.getBrdfLUTTexture()->getPath(), newProbe.getBrdfLUTTexture()->getPath());
}

#endif

int
imstkIBLProbeTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
