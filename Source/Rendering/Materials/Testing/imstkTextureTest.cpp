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

#include "imstkTestingUtils.h"
#include "imstkTexture.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkTextureTest : public TestWithTempFolder
{
protected:
};

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkTextureTest, Serialization)
{
    Texture texture("/test/path/to/texture", Texture::Type::Normal);

    // Serialize
    {
      std::ofstream os(getTempFolder() + "/imstkTexture.cereal", std::ios::binary);
      cereal::JSONOutputArchive archive(os);

      archive(texture);
    }

    // Deserialize
    auto newTexture = Texture();
    {
      std::ifstream is(getTempFolder() + "/imstkTexture.cereal", std::ios::binary);
      cereal::JSONInputArchive dearchive(is);

      dearchive(newTexture);
    }

    EXPECT_EQ(texture.getPath(), newTexture.getPath());
    EXPECT_EQ(texture.getType(), newTexture.getType());
}
#endif

int
imstkTextureTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
