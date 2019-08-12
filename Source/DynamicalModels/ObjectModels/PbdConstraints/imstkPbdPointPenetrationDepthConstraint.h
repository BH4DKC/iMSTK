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

#include "imstkPbdCollisionConstraint.h"
#include <array>
namespace imstk
{
    ///
    /// \brief The PbdPointPenetrationDepthConstraint class for creating non-penetration constraint
    ///  from point to analytical collision response
    ///
    class PbdPointPenetrationDepthConstraint : public PbdCollisionConstraint
    {
    public:
        PbdPointPenetrationDepthConstraint() : PbdCollisionConstraint(1, 0)
        {}

        ///
        /// \brief Returns the type of the pbd collision constraint
        ///
        Type getType() const
        {
            return Type::PointPenetration;
        }

        ///
        /// \brief initialize constraint
        /// \param pIdx1 index of the point from object1
        /// \param penetrationDir is the penetrationDir vector 
        ///
        void initConstraint(std::shared_ptr<PbdModel> model1, const size_t& pIdx1,
             std::array<double,3> penetrationDir);

        ///
        /// \brief
        ///
        bool solvePositionConstraint();

        std::array<double, 3> m_penetrationDir;
    };
}
