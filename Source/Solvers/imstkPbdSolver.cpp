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

#include "imstkPbdObject.h"
#include "imstkPbdSolver.h"
#include "imstkPbdCollisionConstraint.h"
#include "imstkParallelUtils.h"

namespace imstk
{
void
PbdSolver::solve()
{
    m_pbdObject->integratePosition();
    m_pbdObject->solveConstraints();
    resolveCollisionConstraints();
    m_pbdObject->updateVelocity();
}

void
PbdSolver::resolveCollisionConstraints()
{
    if (m_PBDConstraints.size() > 0)
    {
        uint32_t maxIter = 3u;
        uint32_t i       = 0;
        while (++i < maxIter)
        {
            for (const auto constraintList : m_PBDConstraints)
            {
                for (size_t k = 0; k < constraintList->size(); ++k)
                {
                    (*constraintList)[k]->solvePositionConstraint();
                }
            }
        }
    }
}
} // end namespace imstk
