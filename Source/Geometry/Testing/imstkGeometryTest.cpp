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

class imstkGeometryTest : public TestWithTempFolder
{
protected:
    Plane m_geometry; // Can't use imstk::Geometry since pure virtual. Should use google mock class.
};

///
/// \brief TODO
///
TEST_F(imstkGeometryTest, GetSetScaling)
{
    m_geometry.setScaling(2);
    EXPECT_EQ(m_geometry.getScaling(), 2);

    m_geometry.setScaling(0.003);
    EXPECT_EQ(m_geometry.getScaling(), 0.003);

    m_geometry.setScaling(400000000);
    EXPECT_EQ(m_geometry.getScaling(), 400000000);

    m_geometry.setScaling(0);
    EXPECT_GT(m_geometry.getScaling(), 0);

    m_geometry.setScaling(-5);
    EXPECT_GT(m_geometry.getScaling(), 0);
}

///
/// \brief TODO
///
TEST_F(imstkGeometryTest, GetSetTranslation)
{
    auto p1 = Vec3d(12, 0.0005, -400000);
    auto p2 = Vec3d(-500, 30, 0.23);

    m_geometry.setTranslation(p1);
    EXPECT_EQ(m_geometry.getTranslation(), p1);

    m_geometry.setTranslation(p2);
    EXPECT_EQ(m_geometry.getTranslation(), p2);

    m_geometry.setTranslation(p1[0], p1[1], p1[2]);
    EXPECT_EQ(m_geometry.getTranslation(), p1);

    m_geometry.setTranslation(p2[0], p2[1], p2[2]);
    EXPECT_EQ(m_geometry.getTranslation(), p2);
}

///
/// \brief TODO
///
TEST_F(imstkGeometryTest, GetSetRotation)
{
    auto angle1 = 15;
    auto axis1  = Vec3d(12, 0, -0.5);
    auto aa1    = Rotd(angle1, axis1);
    auto q1     = Quatd(aa1);

    auto angle2 = 0.43;
    auto axis2  = Vec3d(4000, -1, 0);
    auto aa2    = Rotd(angle2, axis2);
    auto mat2   = Mat3d(aa2);

    auto angle3 = 800;
    auto axis3  = Vec3d(-0, 100, 2000000);
    auto aa3    = Rotd(angle3, axis3);
    auto mat3   = Mat3d(aa3);

    // NOTE: '==' not defined for Eigen::Quaternion, using 'isApprox'.
    // See https://forum.kde.org/viewtopic.php?f=74&t=118598

    m_geometry.setRotation(q1);
    EXPECT_TRUE(Quatd(m_geometry.getRotation()).isApprox(q1));

    m_geometry.setRotation(mat2);
    EXPECT_TRUE(m_geometry.getRotation().isApprox(mat2));

    m_geometry.setRotation(axis3, angle3);
    EXPECT_TRUE(m_geometry.getRotation().isApprox(mat3));
}

///
/// \brief Serialization
///
#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkGeometryTest, Serialization)
{
    auto p1 = Vec3d(12, 0.0005, -400000);

    auto angle1 = 15;
    auto axis1 = Vec3d(12, 0, -0.5);
    auto aa1 = Rotd(angle1, axis1);
    auto q1 = Quatd(aa1);

    m_geometry.setTranslation(p1);
    m_geometry.setRotation(q1);
    m_geometry.setScaling(2);

    std::shared_ptr<Geometry> geom = std::make_shared<Plane>(m_geometry);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkGeometry.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(geom);
    }

    // Deserialize
    auto newGeometry = std::shared_ptr<Geometry>();
    {
        std::ifstream is(getTempFolder() + "/imstkGeometry.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newGeometry);
    }

    EXPECT_EQ(geom->getTranslation(), newGeometry->getTranslation());
    EXPECT_EQ(geom->getRotation(), newGeometry->getRotation());
    EXPECT_EQ(geom->getScaling(), newGeometry->getScaling());
    EXPECT_EQ(geom->getTotalNumberGeometries(), newGeometry->getTotalNumberGeometries());
    EXPECT_EQ(geom->getGlobalIndex(), newGeometry->getGlobalIndex());
}
#endif

int
imstkGeometryTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
