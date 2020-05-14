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

#include "imstkPlane.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkPlaneTest : public TestWithTempFolder
{
protected:
    Plane m_plane;
};

///
/// \brief TODO
///
TEST_F(imstkPlaneTest, SetGetWidth)
{
    m_plane.setWidth(2);
    EXPECT_EQ(m_plane.getWidth(), 2);

    m_plane.setWidth(0.003);
    EXPECT_EQ(m_plane.getWidth(), 0.003);

    m_plane.setWidth(400000000);
    EXPECT_EQ(m_plane.getWidth(), 400000000);

    m_plane.setWidth(0);
    EXPECT_GT(m_plane.getWidth(), 0);

    m_plane.setWidth(-5);
    EXPECT_GT(m_plane.getWidth(), 0);
}

///
/// \brief TODO
///
TEST_F(imstkPlaneTest, SetGetNormal)
{
    Vec3d n1 = Vec3d(0.2, -0.3, 0.9);
    Vec3d n2 = Vec3d(0.003, -0.001, 0.002);
    Vec3d n3 = Vec3d(400000000, -500000000, 600000000);

    m_plane.setNormal(n1);
    EXPECT_TRUE(m_plane.getNormal().isApprox(n1.normalized()));

    m_plane.setNormal(n2);
    EXPECT_TRUE(m_plane.getNormal().isApprox(n2.normalized()));

    m_plane.setNormal(n3);
    EXPECT_TRUE(m_plane.getNormal().isApprox(n3.normalized()));

    m_plane.setNormal(0, 0, 0);
    EXPECT_FALSE(m_plane.getNormal().isApprox(Vec3d(0, 0, 0)));
}

///
/// \brief TODO
///
TEST_F(imstkPlaneTest, GetVolume)
{
    EXPECT_EQ(m_plane.getVolume(), 0);
}

///
/// \brief Serialization
///
#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkPlaneTest, Serialization)
{
    Vec3d origin = Vec3d(1000.2, -0.3, 0.);
    m_plane.setPosition(origin);
    Vec3d normal = Vec3d(-70.2, 10000.3, 0.00001);
    m_plane.setNormal(normal);
    m_plane.setWidth(3001);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkPlaneTest.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(m_plane);
    }

    // Deserialize
    auto newPlane = Plane();
    {
        std::ifstream is(getTempFolder() + "/imstkPlaneTest.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newPlane);
    }

    EXPECT_EQ(m_plane.getPosition(), newPlane.getPosition());
    EXPECT_EQ(m_plane.getNormal(), newPlane.getNormal());
    EXPECT_EQ(m_plane.getWidth(), newPlane.getWidth());
}
#endif

int
imstkPlaneTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
