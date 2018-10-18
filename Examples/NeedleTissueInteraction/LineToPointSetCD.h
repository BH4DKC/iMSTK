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

#ifndef imstkPointSetToNeedleCD_h
#define imstkPointSetToNeedleCD_h

#include <memory>

#include "imstkCollisionDetection.h"
#include "imstkDeviceTracker.h"

namespace imstk
{
class PointSet;
class DeviceTracker;
class CollisionData;

///
/// \class LineToPointSet
///
/// \brief PointSet to line collision detection 
/// This is a custom implementation that implements needle to tissue
/// collisions
///
class LineToPointSetCD : public CollisionDetection
{
public:

	///
	/// \brief Constructor
	///
    LineToPointSetCD(std::shared_ptr<PointSet>& mesh,
		std::shared_ptr<DeviceTracker> tracker,
		Vec3d startPoint,
        Vec3d endPoint,
		CollisionData& colData) :
		CollisionDetection(CollisionDetection::Type::custom,
			colData),
		m_tracker(tracker),
        m_startPoint(startPoint),
        m_endPoint(endPoint),
        m_pointSet(mesh)
	{
        m_isInContactWithNeedle = std::vector<bool>(m_pointSet->getNumVertices(), false);
        m_colData.clearAll();
    }

	///
	/// \brief Destructor
	///
    ~LineToPointSetCD() { m_colData.clearAll(); };

	///
	/// \brief Detect collision and compute collision data
	///
    void
    LineToPointSetCD::computeCollisionData() override
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

            double tol = m_isSurfaceNode[i]? 0.035 : 0.02;
            
            // add new node to the list of it enters the needle path
            if (!m_isInContactWithNeedle[i] &&  perpendicularDist <= needleLength*tol && m_colData.NeedleColData.size()<1)
            {
                // Check if the projected point is on the needle
                if (distanceAlongLine >= 0 && distanceAlongLine < needleLength && distanceAlongLine / needleLength > 0.95)
                {                      
                    NeedleCollisionData d;
                    d.nodeId = i;
                    d.pointOnNeedle = m_currentStartPoint + needleAxisNormalized*distanceAlongLine;
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
                        contacts.pointOnNeedle = m_currentStartPoint + needleAxisNormalized*distanceAlongLine;
                        break;
                    }
                }
            }
        }
        //std::cout << "Num. of collisions: " << m_colData.NeedleColData.size() << std::endl;
    }

    ///
    /// \brief Set the vector with information whether a node is on surface or not
    ///
    void setSurfaceNodeList(vector<bool>& onSurfaceStatus)
    {
        m_isSurfaceNode = onSurfaceStatus;
    }

private:
    Vec3d m_startPoint, m_endPoint;
    Vec3d m_currentStartPoint, m_currentEndPoint;
    std::shared_ptr<PointSet> m_pointSet;               ///> Vector of PointSet data
	std::shared_ptr<DeviceTracker> m_tracker;           ///> Sphere
    vector<bool> m_isInContactWithNeedle;
    vector<bool> m_isSurfaceNode;
};
}

#endif // ifndef imstkPointSetToNeedleCD_h
