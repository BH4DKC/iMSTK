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

#include "imstkPbdPointPenetrationDepthConstraint.h"
#include "imstkPbdModel.h"

#include "g3log/g3log.hpp"

namespace imstk
{
    void
        PbdPointPenetrationDepthConstraint::initConstraint(std::shared_ptr<PbdModel> model1, const size_t& pIdx1,
            std::array<double, 3> penetrationDir)
    {
        m_model1 = model1;
        m_bodiesFirst[0] = pIdx1;
        m_penetrationDir =  penetrationDir;
    }

    bool
        PbdPointPenetrationDepthConstraint::solvePositionConstraint()
    {
        auto state = m_model1->getCurrentState();
        if (m_bodiesFirst[0] < state->getPositions().size())
        {
            Vec3d & pos = state->getVertexPosition(m_bodiesFirst[0]);
            pos[0] = pos[0] - m_penetrationDir[0] * (1+ m_model1->getParameters()->m_proximity);
            pos[1] = pos[1] - m_penetrationDir[1] * (1 + m_model1->getParameters()->m_proximity);
            pos[2] = pos[2] - m_penetrationDir[2] * (1 + m_model1->getParameters()->m_proximity);
            return true;
        }
        else
            return false;
    }
} // imstk
