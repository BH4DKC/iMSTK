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

#include "imstkPbdConstraint.h"

namespace imstk
{
///
/// \class PbdBendConstraint
///
/// \brief Bend constraint between two segments
///
class PbdBendConstraint : public PbdConstraint
{
public:
    ///
    /// \brief Constructor
    ///
    PbdBendConstraint() : PbdConstraint() { m_vertexIds.resize(3); }

    ///
    /// \brief Returns PBD constraint of type Type::Bend
    ///
    inline Type getType() const override { return Type::Bend; }

    /**
        \brief initConstraint
            p0
               \
                \
                p1
                /
               /
            p2
        \param model
        \param pIdx1 index of p0
        \param pIdx2 index of p1
        \param pIdx3 index of p2
        \param k stiffness
    */
    void initConstraint(PbdModel& model,
                        const size_t& pIdx1, const size_t& pIdx2,
                        const size_t& pIdx3, const double k);

    ///
    /// \brief Solves the bend constraint
    ///
    bool solvePositionConstraint(PbdModel& model) override;

public:
    double m_restLength; ///> Rest length
    double m_stiffness;  ///> Bend stiffness
};
} //imstk
