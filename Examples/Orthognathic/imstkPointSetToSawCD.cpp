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

#include "imstkPointSetToSawCD.h"

#include "imstkCollidingObject.h"
#include "imstkCollisionData.h"
#include "imstkPointSet.h"

#include <g3log/g3log.hpp>

namespace imstk
{
void
PointSetToSawCD::updateBB(Vec3d& maxx, Vec3d& minn)
{
    Vec3d hl(maxx[0]-minn[0], maxx[1] - minn[1], maxx[2] - minn[2]);
    hl = 0.5*hl;

    Vec3d center = minn + (maxx - minn)*0.5;

    Vec3d axis[3];
    axis[0] = Vec3d(1, 0, 0);
    axis[1] = Vec3d(0, 1, 0);
    axis[2] = Vec3d(0, 0, 1);


    StdVectorOfVec3d corners;
    corners.push_back(center - axis[0] * hl[0] - axis[1] * hl[1] - axis[2] * hl[2]);
    corners.push_back(center + axis[0] * hl[0] - axis[1] * hl[1] - axis[2] * hl[2]);
    corners.push_back(center + axis[0] * hl[0] - axis[1] * hl[1] + axis[2] * hl[2]);
    corners.push_back(center - axis[0] * hl[0] - axis[1] * hl[1] + axis[2] * hl[2]);

    corners.push_back(center - axis[0] * hl[0] + axis[1] * hl[1] - axis[2] * hl[2]);
    corners.push_back(center + axis[0] * hl[0] + axis[1] * hl[1] - axis[2] * hl[2]);
    corners.push_back(center + axis[0] * hl[0] + axis[1] * hl[1] + axis[2] * hl[2]);
    corners.push_back(center - axis[0] * hl[0] + axis[1] * hl[1] + axis[2] * hl[2]);

    for (int i = 0; i < 8; ++i)
    {
        m_bladeBB->setVertexPosition(i, corners[i]);
    }
}

void
PointSetToSawCD::computeCollisionData()
{
    // Clear collisionData
    m_colData.clearAll();

    Vec3d devicePosition = m_tracker->getPosition();
    Mat3d deviceOrientation = m_tracker->getRotation().toRotationMatrix();

    const double scalingBlade = 10.0; // should be same as in the main
    const double cylinderRadius = scalingBlade*0.2236;
    const double cylinderHalfLength = scalingBlade*1.0;
    const double halfBladeWidth = scalingBlade*0.2/2;
    Vec3d bladeNormal = deviceOrientation*Vec3d(0., 1., 0.);
    bladeNormal.normalize();
    Vec3d sawAxis = deviceOrientation*Vec3d(0., 0., -1.); // Same as cylinder axis
    sawAxis.normalize();

    // Update OBB, its bounding box and bounding cylinder
    const Vec3d bladeCenter = deviceOrientation*m_bladeOBB->m_center + devicePosition;
    const auto corners = m_bladeOBB->getCorners();
    Vec3d newCorners[8];
    for (int i = 0; i < 8; ++i)
    {
        newCorners[i] = deviceOrientation*corners[i] + devicePosition;
    }

    // find the bounding box of blade's OBB
    Vec3d bbMin(MAX_D, MAX_D, MAX_D), bbMax(-MAX_D, -MAX_D, -MAX_D);
    for (int i = 0; i < 8; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (newCorners[i][j] < bbMin[j])
            {
                bbMin[j] = newCorners[i][j];
            }

            if (newCorners[i][j] > bbMax[j])
            {
                bbMax[j] = newCorners[i][j];
            }
        }
    }

    //updateBB(bbMax, bbMin);

    // Cull points outside the bounding box of the OBB
    size_t nodeId = 0;
    std::vector<size_t> pointList;
    auto vertList = m_pointSet->getVertexPositions();
    for (const auto& p : vertList)
    {
        if (p[0] > bbMin[0] && p[0] < bbMax[0])
        {
            if (p[1] > bbMin[1] && p[1] < bbMax[1])
            {
                if (p[2] > bbMin[2] && p[2] < bbMax[2])
                {
                    pointList.push_back(nodeId);
                }
            }
        }
        nodeId++;
    }

    Vec3d vecAlongWidth = deviceOrientation*Vec3d(1, 0, 0);
    vecAlongWidth.normalize();
    for (const auto& i : pointList)
    {
        // Cull points outside the bounding cylinder of the OBB
        const Vec3d s = vertList[i] - bladeCenter;
        if (abs(s.dot(sawAxis)) < cylinderHalfLength)
        {
            const double distToAxis = (s - sawAxis*s.dot(sawAxis)).norm();
            if (distToAxis < cylinderRadius)
            {
                // Cull points if the distance is below certain threshold to the blade
                if (abs(s.dot(bladeNormal)) < halfBladeWidth)
                {
                    const double depth = cylinderRadius - s.dot(vecAlongWidth);
                    m_colData.MAColData.push_back({ i, vecAlongWidth* -depth });
                }
            }
        }
    }
}
} // imstk
