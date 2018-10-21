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

#include "LineToPointSetCD.h"
#include "imstkDeviceTracker.h"

namespace imstk
{

void
LineToPointSetCD::computeCollisionData()
{
    Vec3d devicePosition = m_tracker->getPosition();
    Mat3d deviceOrientation = m_tracker->getRotation().toRotationMatrix();

    m_currentStartPoint = deviceOrientation*m_startPoint + devicePosition;
    m_currentEndPoint = deviceOrientation*m_endPoint + devicePosition;

    const Vec3d needleAxis = m_currentEndPoint - m_currentStartPoint;
    const Vec3d needleAxisNormalized = needleAxis / needleAxis.norm();
    const double needleLength = (m_endPoint - m_startPoint).norm();

    //nodeDofToIdMap.clear();
    for (int i = 0; i < m_pointSet->getNumVertices(); ++i)
    {
        //if (M.fixed[i]) { continue; }

        Vec3d lineToPoint = m_pointSet->getVertexPosition(i) - m_currentStartPoint;

        double distanceAlongLine = lineToPoint.dot(needleAxisNormalized);
        double perpendicularDist = (lineToPoint - distanceAlongLine *needleAxisNormalized).norm();

        double tol = m_isSurfaceNode[i] ? 0.035 : 0.02;

        // add new node to the list of it enters the needle path
        if (!m_isInContactWithNeedle[i] && perpendicularDist <= needleLength*tol && m_colData.NeedleColData.size() < 1)
        {
            // Check if the projected point is on the needle
            if (distanceAlongLine >= 0 && distanceAlongLine < needleLength && distanceAlongLine / needleLength > 0.95)
            {
                NeedleCollisionData d;
                d.nodeId = i;
                d.pointOnNeedle = m_currentStartPoint + needleAxisNormalized*distanceAlongLine;
                d.prevPos = d.pointOnNeedle;
                d.axis = needleAxisNormalized;
                m_colData.NeedleColData.push_back(d);

                m_isInContactWithNeedle[i] = true;
            }
        }

        // If the node is in the contact list and falls outside the needle, remove from contact list
        if (m_isInContactWithNeedle[i] && (distanceAlongLine < 0 || distanceAlongLine > needleLength*1.01))
        {
            int count = 0;
            for (const auto&contacts : m_colData.NeedleColData)
            {
                if (contacts.nodeId == i)
                {
                    m_colData.NeedleColData.erase(m_colData.NeedleColData.begin() + count);
                    break;
                }
                count++;
            }
            m_isInContactWithNeedle[i] = false;
        }
        else // projected point still on the needle, so update the projection point
        {
            for (auto&contacts : m_colData.NeedleColData)
            {
                if (contacts.nodeId == i)
                {
                    contacts.prevPos = contacts.pointOnNeedle;
                    contacts.pointOnNeedle = m_currentStartPoint + needleAxisNormalized*distanceAlongLine;
                    break;
                }
            }
        }
    }
    //std::cout << "Num. of collisions: " << m_colData.NeedleColData.size() << std::endl;
}

} // imstk
