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

#include "imstkSerialize.h"
#include "imstkCollisionDetection.h"
#include "imstkCylinder.h"
#include "imstkSphere.h"

namespace imstk
{
class Sphere;
struct CollisionData;

///
/// \class SphereToCylinderCD
///
/// \brief Sphere-Cylinder collision detection
///
class SphereToCylinderCD : public CollisionDetection
{
public:

    ///
    /// \brief Constructor
    ///
    SphereToCylinderCD(std::shared_ptr<Sphere>        sphere,
                       std::shared_ptr<Cylinder>      cylinder,
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
            iMSTK_SERIALIZE_SUPERCLASS(CollisionDetection),
            iMSTK_SERIALIZE(sphere),
            iMSTK_SERIALIZE(cylinder)
        );
    }
#endif

private:
    std::shared_ptr<Cylinder> m_cylinder;
    std::shared_ptr<Sphere>   m_sphere;
};
}
