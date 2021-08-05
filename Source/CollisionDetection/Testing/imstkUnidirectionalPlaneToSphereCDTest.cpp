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

#include "imstkUnidirectionalPlaneToSphereCD.h"
#include "imstkSphere.h"
#include "imstkPlane.h"

using namespace imstk;

TEST(imstkUnidirectionalPlaneToSphereCDTest, IntersectionTestAB)
{
    UnidirectionalPlaneToSphereCD unidirectionalPlaneToSphereCD;

    auto sphere = std::make_shared<Sphere>(Vec3d(0.0, 0.0, 0.0), 1.0);
    auto plane  = std::make_shared<Plane>();

    unidirectionalPlaneToSphereCD.setInput(sphere, 0);
    unidirectionalPlaneToSphereCD.setInput(plane, 1);
    unidirectionalPlaneToSphereCD.setGenerateCD(true, true); // Generate both A and B
    unidirectionalPlaneToSphereCD.update();

    std::shared_ptr<CollisionData> colData = unidirectionalPlaneToSphereCD.getCollisionData();

    // Should be one element per side
    EXPECT_EQ(1, colData->elementsA.getSize());
    EXPECT_EQ(1, colData->elementsB.getSize());

    // That element should be a point directional element
    EXPECT_EQ(CollisionElementType::PointDirection, colData->elementsA[0].m_type);
    EXPECT_EQ(CollisionElementType::PointDirection, colData->elementsB[0].m_type);

    // Sphere A (which is below B), should be pushed down, while sphere B (above A), should be pushed up
    EXPECT_EQ(Vec3d(0.0, 1.0, 0.0), colData->elementsA[0].m_element.m_PointDirectionElement.dir);
    EXPECT_EQ(Vec3d(0.0, -1.0, 0.0), colData->elementsB[0].m_element.m_PointDirectionElement.dir);

    // Should have depths of 1.0
    EXPECT_EQ(1.0, colData->elementsA[0].m_element.m_PointDirectionElement.penetrationDepth);
    EXPECT_EQ(1.0, colData->elementsB[0].m_element.m_PointDirectionElement.penetrationDepth);

    // Contacts should be on the surface of spheres
    EXPECT_EQ(Vec3d(0.0, -1.0, 0.0), colData->elementsA[0].m_element.m_PointDirectionElement.pt);
    EXPECT_EQ(Vec3d(0.0, 0.0, 0.0), colData->elementsB[0].m_element.m_PointDirectionElement.pt);
}

TEST(imstkUnidirectionalPlaneToSphereCDTest, NonIntersectionTestAB)
{
    UnidirectionalPlaneToSphereCD unidirectionalPlaneToSphereCD;

    auto sphere = std::make_shared<Sphere>(Vec3d(0.0, 0.0, 0.0), 0.5);
    auto plane  = std::make_shared<Plane>(Vec3d(0.0, -0.75, 0.0), Vec3d(0.0, 1.0, 0.0));

    unidirectionalPlaneToSphereCD.setInput(sphere, 0);
    unidirectionalPlaneToSphereCD.setInput(plane, 1);
    unidirectionalPlaneToSphereCD.setGenerateCD(true, true); // Generate both A and B
    unidirectionalPlaneToSphereCD.update();

    std::shared_ptr<CollisionData> colData = unidirectionalPlaneToSphereCD.getCollisionData();

    // Should be no elements
    EXPECT_EQ(0, colData->elementsA.getSize());
    EXPECT_EQ(0, colData->elementsB.getSize());
}
