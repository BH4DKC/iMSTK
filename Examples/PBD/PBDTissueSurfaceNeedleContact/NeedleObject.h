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

#include "imstkRigidObject2.h"

using namespace imstk;

class NeedleObject : public RigidObject2
{
public:
    NeedleObject(const std::string& name) : RigidObject2(name) { }
    virtual ~NeedleObject() = default;

    virtual const std::string getTypeName() const override { return "NeedleObject"; }

public:
    void setInserted(const bool inserted) { m_inserted = inserted; }
    bool getInserted() const { return m_inserted; }

    ///
    /// \brief Returns the axes of the needle (tip-tail)
    ///
    const Vec3d getNeedleAxes() const
    {
        return (-getCollidingGeometry()->getRotation().col(1)).normalized();
    }

protected:
    bool m_inserted = false;
};