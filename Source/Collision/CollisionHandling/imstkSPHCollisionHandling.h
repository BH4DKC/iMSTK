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
#include "imstkCollisionHandling.h"
#include "imstkSPHObject.h"

namespace imstk
{
class SPHCollisionHandling : public CollisionHandling
{
public:
    SPHCollisionHandling(const Side& side, const std::shared_ptr<CollisionData>& colData,
                         const std::shared_ptr<CollidingObject>& obj);

    SPHCollisionHandling() = delete;

    ///
    /// \brief Compute forces based on collision data
    ///
    virtual void processCollisionData() override;

private:
    std::shared_ptr<SPHObject> m_SPHObject;
};
} // end namespace imstk
