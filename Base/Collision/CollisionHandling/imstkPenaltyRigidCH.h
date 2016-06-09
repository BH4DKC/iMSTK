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

#ifndef imstkPenaltyRigidCH_h
#define imstkPenaltyRigidCH_h

#include "imstkCollisionHandling.h"

#include <memory>

namespace imstk {

class CollidingObject;
class CollisionData;

class PenaltyRigidCH : public CollisionHandling
{
public:

    ///
    /// \brief Constructor
    ///
    PenaltyRigidCH(std::shared_ptr<CollidingObject> obj,
                   CollisionData& colData) :
        CollisionHandling(CollisionHandling::Type::Penalty),
        m_obj(obj),
        m_colData(colData)
    {}

    ///
    /// \brief Destructor
    ///
    ~PenaltyRigidCH() = default;

    ///
    /// \brief Compute forces based on collision data (pure virtual)
    ///
    void computeContactForces() override;

private:

    std::shared_ptr<CollidingObject> m_obj;
    CollisionData& m_colData;

};
}

#endif // ifndef imstkPenaltyRigidCH_h
