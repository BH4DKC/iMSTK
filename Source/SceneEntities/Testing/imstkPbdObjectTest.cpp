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
#include "imstkPbdObject.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkPbdObjectTest : public TestWithTempFolder
{
protected:
    PbdObject m_obj;
};

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkPbdObjectTest, Serialization)
{
    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkPbdObject.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(m_obj);
    }

    // Deserialize
    auto newObj = PbdObject();
    {
        std::ifstream is(getTempFolder() + "/imstkPbdObject.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newObj);
    }

    EXPECT_EQ(m_obj.getName(), newObj.getName());
}
#endif

int
imstkPbdObjectTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
