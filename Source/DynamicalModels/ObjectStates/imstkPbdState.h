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

#include <Eigen/Dense>
#include <vector>

#include "imstkPointSet.h"
#include "imstkMath.h"

namespace imstk
{
///
/// \class PbdState
///
/// \brief State of the body governed by PBD mathematical model
///
class PbdState
{
public:
    ///
    /// \brief Default constructor/destructor
    ///
    PbdState()  = default;
    ~PbdState() = default;

    ///
    /// \brief Initialize the pbd state
    ///
    void initialize(const size_t numNodes, const bool (&options)[3]);
    void initialize(const std::shared_ptr<PointSet>& m, const bool(&options)[3]);

    ///
    /// \brief Get/Set nodal position given the index
    ///
    void setVertexPosition(const size_t& idx, const Vec3d& pos) { m_pos.at(idx) = pos; }
    Vec3d& getVertexPosition(const size_t& idx) { return m_pos.at(idx); }

    ///
    /// \brief Returns the vector of current nodal positions
    ///
    StdVectorOfVec3d& getPositions() { return m_pos; }
    void setPositions(const StdVectorOfVec3d& p) { m_pos = p; }

    ///
    /// \brief Returns the vector of current nodal velocities
    ///
    StdVectorOfVec3d& getVelocities() { return m_vel; }

    ///
    /// \brief Returns the vector of current nodal accelerations
    ///
    StdVectorOfVec3d& getAccelerations() { return m_acc; }

    ///
    /// \brief Set the state to a given one
    ///
    void setState(std::shared_ptr<PbdState> rhs);

private:
    StdVectorOfVec3d m_pos; ///> Nodal positions
    StdVectorOfVec3d m_vel; ///> Nodal velocities
    StdVectorOfVec3d m_acc; ///> Nodal acelerations
};
} // imstk
