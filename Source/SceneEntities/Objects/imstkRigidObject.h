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

#include "imstkDynamicObject.h"

namespace imstk
{
class RigidBodyModel;

///
/// \class RigidObject
///
/// \brief Scene objects that are governed by rigid body dynamics
///
class RigidObject : public DynamicObject
{
public:
    explicit RigidObject(const std::string& name) : DynamicObject(name)
    {
        m_type = Type::Rigid;
    }

    virtual ~RigidObject() = default;

public:
    ///
    /// \brief Initialize the rigid scene object
    ///
    bool initialize() override;

    ///
    /// \brief Add local force at a position relative to object
    ///
    void addForce(const Vec3d& force, const Vec3d& pos, bool wakeup = true);

    std::shared_ptr<RigidBodyModel> getRigidBodyModel() const { return m_rigidBodyModel; }

protected:
    std::shared_ptr<RigidBodyModel> m_rigidBodyModel;
};
} // imstk