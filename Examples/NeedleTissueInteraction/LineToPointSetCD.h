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

#include <vector>

#include "imstkCollisionDetection.h"
#include "imstkCollisionData.h"
#include "imstkPointSet.h"
#include "imstkMath.h"

namespace imstk
{
class DeviceTracker;

enum slipState
{
    sliding,
    sticking
};

struct constrainedNode
{
    Vec3d prevPos;
    double greyScaleValue;
    slipState slipStatus;
    bool isPiercingNode;
};

enum needleMotionState
{
    noMovement,
    insertion,
    retraction
};

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
    void computeCollisionData();

    ///
    /// \brief Set the vector with information whether a node is on surface or not
    ///
    void setSurfaceNodeList(std::vector<bool>& onSurfaceStatus)
    {
        m_isSurfaceNode = onSurfaceStatus;
    }

    ///
    /// \brief Needle axis
    ///
    Vec3d getNeedleAxis() { return m_needleAxis; };

    ///
    /// \brief 
    ///
    void computeNeedleMotionDirection();

    ///
    /// \brief 
    ///
    unsigned int getNeedleState() { return m_needleInsersion; };

private:
    Vec3d m_startPoint, m_endPoint;
    Vec3d m_currentStartPoint, m_currentEndPoint;
    Vec3d m_prevStartPoint;
    std::shared_ptr<PointSet> m_pointSet;               ///> Vector of PointSet data
	std::shared_ptr<DeviceTracker> m_tracker;           ///> Sphere
    std::vector<bool> m_isInContactWithNeedle;
    std::vector<bool> m_isSurfaceNode;
    needleMotionState m_needleInsersion = needleMotionState::noMovement;
    Vec3d m_needleAxis;
};
}

#endif // ifndef imstkPointSetToNeedleCD_h
