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

    const double cylinderRadius = 10*0.25;
    const double bladeWidth = 10*0.1;
    Vec3d bladeNormal = deviceOrientation*Vec3d(0., 1., 0.);
    bladeNormal.normalize();
    Vec3d sawAxis = deviceOrientation*Vec3d(0., 0., -1.); // Same as cylinder axis
    sawAxis.normalize();

    // Update OBB, its bounding box and bounding cylinder
    auto newCenter = deviceOrientation*m_bladeOBB->m_center + devicePosition;
    auto corners = m_bladeOBB->getCorners();
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

    // Cull points outside the bounding cylinder of the OBB
    for (const auto& i : pointList)
    {
        // Do the actual check
        Vec3d q = vertList[i];
        Vec3d s = q - newCenter;
        double scale = s.dot(sawAxis);
        Vec3d perpendicular = s - sawAxis*scale;

        if (perpendicular.norm() < cylinderRadius)
        {
            // Cull points if the distance is below certain threshold to the blade
            double distanceToBlade = bladeNormal.dot(s);
            if (distanceToBlade < bladeWidth / 2.)
            {
                 double depth = cylinderRadius - (s - distanceToBlade*bladeNormal).norm();
                 Vec3d dire = deviceOrientation*Vec3d(1, 0, 0);
                 m_colData.MAColData.push_back({ i, dire*depth });
            }
        }
    }
}
} // imstk
