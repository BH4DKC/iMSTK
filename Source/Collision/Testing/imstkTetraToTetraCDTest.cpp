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

#include  <memory>

#include "imstkCollisionData.h"
#include "imstkIsometricMap.h"
#include "imstkMeshIO.h"
#include "imstkTetraToTetraCD.h"
#include "imstkTetrahedralMesh.h"

using namespace imstk;

class imstkTetraToTetraCDTest : public ::testing::Test
{
protected:
    TetraToTetraCD* m_CD;
};

std::shared_ptr<TetrahedralMesh>
loadMesh(std::string externalDataSuffix)
{
    std::string                      file = iMSTK_DATA_ROOT + externalDataSuffix;
    std::shared_ptr<TetrahedralMesh> volMesh
        = std::static_pointer_cast<TetrahedralMesh>(imstk::MeshIO::read(file));
    if (!volMesh)
    {
        LOG(FATAL) << "Failed to read a volumetric mesh file : " << file;
    }
    return volMesh;
}

std::shared_ptr<TetrahedralMesh>
duplicate(std::shared_ptr<TetrahedralMesh> mesh)
{
    return std::make_shared<TetrahedralMesh>(*mesh.get());
}

TEST_F(imstkTetraToTetraCDTest, NoSelfIntersection)
{
    std::shared_ptr<TetrahedralMesh> a = loadMesh("/asianDragon/asianDragon.veg");
    auto                             b = std::make_shared<TetrahedralMesh>(TetrahedralMesh()); //empty mesh

    auto cd = std::make_shared<CollisionData>();

    m_CD = new TetraToTetraCD(a, b, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 0);

    m_CD = new TetraToTetraCD(b, a, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 0);
}

TEST_F(imstkTetraToTetraCDTest, IntersectionThenNoIntersection1T)
{
    std::shared_ptr<TetrahedralMesh> a = loadMesh("/oneTet/oneTet.veg");
    auto                             b = duplicate(a);

    b->translateVertices(imstk::Vec3d(0.0, 1.0, 2.5));

    auto cd = std::make_shared<CollisionData>();
    m_CD = new TetraToTetraCD(a, b, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 1);
    EXPECT_EQ(cd->PTColData[0].collisionType, PointTetrahedronCollisionDataElement::bPenetratingA);
    EXPECT_EQ(cd->PTColData[0].vertexIdx, 0);
    EXPECT_EQ(cd->PTColData[0].tetreahedronIdx, 0);

    m_CD = new TetraToTetraCD(b, a, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 1);
    EXPECT_EQ(cd->PTColData[0].collisionType, PointTetrahedronCollisionDataElement::aPenetratingB);
    EXPECT_EQ(cd->PTColData[0].vertexIdx, 0);
    EXPECT_EQ(cd->PTColData[0].tetreahedronIdx, 0);

    //now translate b more so there is no intersection
    b->translateVertices(imstk::Vec3d(0.0, 2.0, 0.0));

    m_CD = new TetraToTetraCD(b, a, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 0);

    m_CD = new TetraToTetraCD(a, b, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 0);
}

TEST_F(imstkTetraToTetraCDTest, IntersectionThenNoIntersectionHuman)
{
    std::shared_ptr<TetrahedralMesh> a = loadMesh("/human/human.veg");
    auto                             b = duplicate(a);

    b->translateVertices(imstk::Vec3d(16.0, 0.0, 1.0));

    auto cd = std::make_shared<CollisionData>();

    m_CD = new TetraToTetraCD(a, b, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 4);

    m_CD = new TetraToTetraCD(b, a, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 4);

    //this additional translation produces a different intersection
    b->translateVertices(imstk::Vec3d(0.0, 0.0, 0.5));

    m_CD = new TetraToTetraCD(a, b, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 1);
    EXPECT_EQ(cd->PTColData[0].collisionType, PointTetrahedronCollisionDataElement::aPenetratingB);
    EXPECT_EQ(cd->PTColData[0].vertexIdx, 81);
    EXPECT_EQ(cd->PTColData[0].tetreahedronIdx, 367);

    m_CD = new TetraToTetraCD(b, a, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 1);
    EXPECT_EQ(cd->PTColData[0].collisionType, PointTetrahedronCollisionDataElement::bPenetratingA);
    EXPECT_EQ(cd->PTColData[0].vertexIdx, 81);
    EXPECT_EQ(cd->PTColData[0].tetreahedronIdx, 367);

    //now translate b more so there is no intersection
    b->translateVertices(imstk::Vec3d(0.0, 0.0, 1.0));
    m_CD = new TetraToTetraCD(a, b, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 0);
}

TEST_F(imstkTetraToTetraCDTest, IntersectionOfDifferentMeshes)
{
    std::shared_ptr<TetrahedralMesh> a = loadMesh("/asianDragon/asianDragon.veg");
    std::shared_ptr<TetrahedralMesh> b = loadMesh("/human/human.veg");

    auto cd = std::make_shared<CollisionData>();
    m_CD = new TetraToTetraCD(a, b, cd);
    m_CD->computeCollisionData();
    EXPECT_EQ(cd->PTColData.getSize(), 595);
}

int
imstkTetraToTetraCDTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
