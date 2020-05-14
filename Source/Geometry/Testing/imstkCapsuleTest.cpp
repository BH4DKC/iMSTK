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

#include "imstkMath.h"
#include "imstkCapsule.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkCapsuleTest : public TestWithTempFolder
{
protected:
    Capsule m_capsule;
};

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkCapsuleTest, Serialization)
{
    m_capsule.setRadius(PI);
    m_capsule.setLength(5.0);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkCapsuleTest.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(m_capsule);
    }

    // Deserialize
    auto newCapsule = Capsule();
    {
        std::ifstream is(getTempFolder() + "/imstkCapsuleTest.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newCapsule);
    }

    EXPECT_EQ(m_capsule.getRadius(), newCapsule.getRadius());
    EXPECT_EQ(m_capsule.getLength(), newCapsule.getLength());
}
#endif

int
imstkCapsuleTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
