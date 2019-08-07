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
#include "imstkGeometryMap.h"
#include "imstkPbdModel.h"

#include <g3log/g3log.hpp>

namespace imstk
{
bool
PbdObject::initialize()
{
    m_pbdModel = std::dynamic_pointer_cast<PbdModel>(m_dynamicalModel);
    if (m_pbdModel)
    {
        return DynamicObject::initialize();
    }
    else
    {
        LOG(FATAL) << "Dynamics pointer cast failure in PbdObject::initialize()";
        return false;
    }
}

void
PbdObject::integratePosition()
{
    if (m_pbdModel && m_pbdModel->hasConstraints())
    {
        m_pbdModel->integratePosition();
    }
}

void
PbdObject::updateVelocity()
{
    if (m_pbdModel && m_pbdModel->hasConstraints())
    {
        m_pbdModel->updateVelocity();
    }
}

void
PbdObject::solveConstraints()
{
    if (m_pbdModel && m_pbdModel->hasConstraints())
    {
        m_pbdModel->projectConstraints();
    }
}

void
PbdObject::reset()
{
    DynamicObject<PbdState>::reset();
    this->updateVelocity();
}
} //imstk