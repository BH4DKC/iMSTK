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

#ifndef imstkPointSetToSawCD_h
#define imstkPointSetToSawCD_h

#include <memory>

#include "imstkCollisionDetection.h"
#include "imstkDeviceTracker.h"
#include "imstkOBB.h"
#include "imstkSurfaceMesh.h"

namespace imstk
{
class PointSet;
class DeviceTracker;
class CollisionData;

///
/// \class PointSetToSphereCD
///
/// \brief PointSet to saw collision detection
///
class PointSetToSawCD : public CollisionDetection
{
public:

    ///
    /// \brief Constructor
    ///
    PointSetToSawCD(std::vector<std::shared_ptr<PointSet>>& pointSet,
                    std::shared_ptr<DeviceTracker> tracker,
                    std::shared_ptr<OBB> baldeOBB,
                    std::shared_ptr<SurfaceMesh> baldeBB,
                    CollisionData& colData) :
        CollisionDetection(CollisionDetection::Type::custom,
                           colData),
        m_bladeOBB(baldeOBB),
        m_bladeBB(baldeBB),
        m_tracker(tracker)
    {
        m_pointSet = pointSet;
        m_BBmin.resize(m_pointSet.size());
        m_BBmax.resize(m_pointSet.size());
        m_BBOverlapped.resize(m_pointSet.size());

        for (int i = 0; i < m_pointSet.size(); ++i)
        {
            m_pointSet[i]->computeBoundingBox(m_BBmin[i], m_BBmax[i]);
        }
    }

    ///
    /// \brief Destructor
    ///
    ~PointSetToSawCD() = default;

    void updateToolBoundingBox(Vec3d& maxx, Vec3d& minn);

    ///
    /// \brief Detect collision and compute collision data
    ///
    void computeCollisionData() override;

private:
    std::vector<Vec3d> m_BBmin;
    std::vector<Vec3d> m_BBmax;
    std::vector<bool> m_BBOverlapped;

    std::vector<shared_ptr<PointSet>> m_pointSet;       ///> Vector of PointSet data
    std::shared_ptr<OBB> m_bladeOBB;                    ///> OBB the sphere in the rest configuration
    std::shared_ptr<SurfaceMesh> m_bladeBB;             ///> BB of the OBB
    std::shared_ptr<DeviceTracker> m_tracker;           ///> Sphere
};
}

#endif // ifndef PointSetToSawCD_h
