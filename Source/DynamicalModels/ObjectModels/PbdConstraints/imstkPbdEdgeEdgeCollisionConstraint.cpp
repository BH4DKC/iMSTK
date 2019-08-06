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

#include "imstkPbdEdgeEdgeCollisionConstraint.h"
#include "imstkPbdModel.h"

#include "g3log/g3log.hpp"

namespace imstk
{
void
PbdEdgeEdgeConstraint::initConstraint(std::shared_ptr<PbdModel> model1,
                                      const size_t& pIdx1, const size_t& pIdx2,
                                      std::shared_ptr<PbdModel> model2,
                                      const size_t& pIdx3, const size_t& pIdx4)
{
    m_model1          = model1;
    m_model2          = model2;
    m_bodiesFirst[0]  = pIdx1;
    m_bodiesFirst[1]  = pIdx2;
    m_bodiesSecond[0] = pIdx3;
    m_bodiesSecond[1] = pIdx4;
}

bool
PbdEdgeEdgeConstraint::solvePositionConstraint()
{
    const auto i0 = m_bodiesFirst[0];
    const auto i1 = m_bodiesFirst[1];
    const auto i2 = m_bodiesSecond[0];
    const auto i3 = m_bodiesSecond[1];

    auto state1 = m_model1->getCurrentState();
    auto state2 = m_model2->getCurrentState();

    Vec3d& x0 = state1->getVertexPosition(i0);
    Vec3d& x1 = state1->getVertexPosition(i1);
    Vec3d& x2 = state2->getVertexPosition(i2);
    Vec3d& x3 = state2->getVertexPosition(i3);

    auto a = (x3 - x2).dot(x1 - x0);
    auto b = (x1 - x0).dot(x1 - x0);
    auto c = (x0 - x2).dot(x1 - x0);
    auto d = (x3 - x2).dot(x3 - x2);
    auto e = a;
    auto f = (x0 - x2).dot(x3 - x2);

    auto   det = a * e - d * b;
    double s   = 0.5;
    double t   = 0.5;
    if (fabs(det) > 1e-12)
    {
        s = (c * e - b * f) / det;
        t = (c * d - a * f) / det;
        if (s < 0 || s > 1.0 || t < 0 || t > 1.0)
        {
            return false;
        }
    }
    else
    {
        //LOG(WARNING) << "det is null";
    }

    Vec3d P = x0 + t * (x1 - x0);
    Vec3d Q = x2 + s * (x3 - x2);

    Vec3d n = Q - P;
    auto  l = n.norm();
    n /= l;

    const auto dist = m_model1->getParameters()->m_proximity + m_model2->getParameters()->m_proximity;

    if (l > dist)
    {
        return false;
    }

    Vec3d grad0 = -(1 - t) * n;
    Vec3d grad1 = -(t) * n;
    Vec3d grad2 = (1 - s) * n;
    Vec3d grad3 = (s) * n;

    const auto im0 = m_model1->getInvMass(i0);
    const auto im1 = m_model1->getInvMass(i1);
    const auto im2 = m_model2->getInvMass(i2);
    const auto im3 = m_model2->getInvMass(i3);

    auto lambda = im0 * grad0.squaredNorm() +
                  im1 * grad1.squaredNorm() +
                  im2 * grad2.squaredNorm() +
                  im3 * grad3.squaredNorm();

    lambda = (l - dist) / lambda;

//    LOG(INFO) << "Lambda:" << lambda <<" Normal:" << n[0] <<" " << n[1] <<" "<<n[2];

    if (im0 > 0)
    {
        x0 += -im0* lambda* grad0* m_model1->getParameters()->m_contactStiffness;
    }

    if (im1 > 0)
    {
        x1 += -im1* lambda* grad1* m_model1->getParameters()->m_contactStiffness;
    }

    if (im2 > 0)
    {
        x2 += -im2* lambda* grad2* m_model2->getParameters()->m_contactStiffness;
    }

    if (im3 > 0)
    {
        x3 += -im3* lambda* grad3* m_model2->getParameters()->m_contactStiffness;
    }

    return true;
}
} // imstk
