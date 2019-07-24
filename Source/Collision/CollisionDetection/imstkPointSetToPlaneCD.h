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

#pragma once

#include "imstkCollisionDetection.h"
#include <memory>

namespace imstk
{
class PointSet;
class Plane;
class Geometry;
struct CollisionData;

///
/// \class PointSetToPlaneCD
///
/// \brief PointSet to plane collision detection
///
class PointSetToPlaneCD : public CollisionDetection
{
public:

    ///
    /// \brief Constructor
    ///
    PointSetToPlaneCD(std::shared_ptr<PointSet>      pointSet,
                      std::shared_ptr<Plane>         plane,
                      std::shared_ptr<CollisionData> colData);

    ///
    /// \brief Detect collision and compute collision data
    ///
    void computeCollisionData() override;

private:
    std::shared_ptr<PointSet> m_pointSet;
    std::shared_ptr<Plane>    m_plane;
};
}
