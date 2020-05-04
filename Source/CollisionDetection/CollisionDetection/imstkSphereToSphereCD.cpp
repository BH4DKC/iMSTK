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

#include "imstkSphereToSphereCD.h"
#include "imstkNarrowPhaseCD.h"
#include "imstkCollisionData.h"

namespace imstk
{
SphereToSphereCD::SphereToSphereCD(std::shared_ptr<Sphere>        sphereA,
                                   std::shared_ptr<Sphere>        sphereB,
                                   std::shared_ptr<CollisionData> colData) :
    CollisionDetection(CollisionDetection::Type::SphereToSphere, colData),
    m_sphereA(sphereA),
    m_sphereB(sphereB)
{
}

void
SphereToSphereCD::computeCollisionData()
{
    m_colData->clearAll();
    NarrowPhaseCD::sphereToSphere(m_sphereA.get(), m_sphereB.get(), m_colData);
}
}
