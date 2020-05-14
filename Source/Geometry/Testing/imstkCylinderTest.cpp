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
#include "imstkCylinder.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkCylinderTest : public TestWithTempFolder
{
protected:
    Cylinder m_cylinder;
};

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkCylinderTest, Serialization)
{
    m_cylinder.setRadius(PI);
    m_cylinder.setLength(5.0);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkCylinderTest.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(m_cylinder);
    }

    // Deserialize
    auto newCylinder = Cylinder();
    {
        std::ifstream is(getTempFolder() + "/imstkCylinderTest.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newCylinder);
    }

    EXPECT_EQ(m_cylinder.getRadius(), newCylinder.getRadius());
    EXPECT_EQ(m_cylinder.getLength(), newCylinder.getLength());
}
#endif

int
imstkCylinderTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
