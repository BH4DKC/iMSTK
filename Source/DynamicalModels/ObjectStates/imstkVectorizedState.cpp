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

#include "imstkVectorizedState.h"

namespace imstk
{
void
VectorizedState::initialize(const size_t numDof)
{
    m_q.resize(numDof);
    m_qDot.resize(numDof);
    m_qDotDot.resize(numDof);

    m_q.setZero();
    m_qDot.setZero();
    m_qDotDot.setZero();
};

void
VectorizedState::setState(const Vectord& u, const Vectord& v, const Vectord& a)
{
    m_q       = u;
    m_qDot    = v;
    m_qDotDot = a;
}

void
VectorizedState::setU(const Vectord& u)
{
    m_q = u;
}

void
VectorizedState::setV(const Vectord& v)
{
    m_qDot = v;
}

void
VectorizedState::setA(const Vectord& a)
{
    m_qDotDot = a;
}

void
VectorizedState::setState(std::shared_ptr<VectorizedState> rhs)
{
    m_q       = rhs->getQ();
    m_qDot    = rhs->getQDot();
    m_qDotDot = rhs->getQDotDot();
}
} // imstk