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

#include "imstkMacros.h"
#include "imstkPbdCollisionHandling.h"

#include "NeedleObject.h"

using namespace imstk;

///
/// \brief Surface collision disabled upon puncture
///
class NeedlePbdCH : public PbdCollisionHandling
{
public:
    NeedlePbdCH() = default;
    ~NeedlePbdCH() override = default;

    IMSTK_TYPE_NAME(NeedlePbdCH)

protected:
    void handle(
        const std::vector<CollisionElement>& elementsA,
        const std::vector<CollisionElement>& elementsB) override
    {
        PbdCollisionHandling::handle(elementsA, elementsB);

        // Do embedding update here
    }

    ///
    /// \brief Add a vertex-triangle constraint
    ///
    void addVTConstraint(
        VertexMassPair ptA,
        VertexMassPair ptB1, VertexMassPair ptB2, VertexMassPair ptB3,
        double stiffnessA, double stiffnessB) override
    {
        auto needleObj = std::dynamic_pointer_cast<NeedleObject>(getInputObjectB());
        //if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {
            PbdCollisionHandling::addVTConstraint(ptA, ptB1, ptB2, ptB3, stiffnessA, stiffnessB);
        }
    }
};