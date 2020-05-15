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
#include "imstkSphere.h"
#include "imstkIdentityMap.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkIdentityMapTest : public TestWithTempFolder
{
public:
    imstkIdentityMapTest()
    {
        m_master = std::make_shared<Sphere>("Master");
        m_master->setPosition(Vec3d(0, 0, 0));
        m_master->setRadius(1.0);

        m_puppet = std::make_shared<Sphere>("Puppet");
        m_puppet->setPosition(Vec3d(5, 0, 0));
        m_puppet->setRadius(1.0);
    }

protected:
    std::shared_ptr<Sphere> m_master;
    std::shared_ptr<Sphere> m_puppet;

    IdentityMap m_map;
};

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkIdentityMapTest, Serialization)
{
    m_map.setMaster(m_master);
    m_map.setSlave(m_puppet);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkIdentityMapTest.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(m_map);
    }

    // Deserialize
    auto newMap = IdentityMap();
    {
        std::ifstream is(getTempFolder() + "/imstkIdentityMapTest.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newMap);
    }

    EXPECT_EQ(m_map.getMaster()->getName(), newMap.getMaster()->getName());
}

TEST_F(imstkIdentityMapTest, SerializationSameObject)
{
    m_map.setMaster(m_master);
    m_map.setSlave(m_master);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkIdentityMapTest.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(m_map);
    }

    // Deserialize
    auto newMap = IdentityMap();
    {
        std::ifstream is(getTempFolder() + "/imstkIdentityMapTest.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newMap);
    }

    EXPECT_EQ(m_map.getMaster()->getName(), newMap.getSlave()->getName());
}
#endif

int
imstkIdentityMapTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
