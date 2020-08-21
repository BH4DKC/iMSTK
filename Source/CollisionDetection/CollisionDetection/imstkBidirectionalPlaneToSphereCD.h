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
#include "imstkSerialize.h"
#include "imstkSphere.h"
#include "imstkPlane.h"

namespace imstk
{
struct CollisionData;

///
/// \class BidirectionalPlaneToSphereCD
///
/// \brief Plane to sphere collision detection
///
class BidirectionalPlaneToSphereCD : public CollisionDetection
{
public:
    ///
    /// \brief Constructor
    ///
    BidirectionalPlaneToSphereCD(std::shared_ptr<Plane>         planeA,
                                 std::shared_ptr<Sphere>        sphereB,
                                 std::shared_ptr<CollisionData> colData);

    ///
    /// \brief Detect collision and compute collision data
    ///
    void computeCollisionData() override;

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive, std::uint32_t const version)
    {
        UNUSED(version);
        archive(
            iMSTK_SERIALIZE_SUPERCLASS(CollisionDetectionS),
            iMSTK_SERIALIZE(planeA),
            iMSTK_SERIALIZE(planeB)
        );
    }
#endif

private:
    std::shared_ptr<Plane>  m_planeA;
    std::shared_ptr<Sphere> m_sphereB;
};
} // namespace imstk
