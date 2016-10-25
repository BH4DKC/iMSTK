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

#ifndef IMSTK_PBD_DIHEDRAL_CONSTRAINT_H
#define IMSTK_PBD_DIHEDRAL_CONSTRAINT_H

#include "imstkPbdConstraint.h"

namespace imstk
{

///
/// \class DihedralConstraint
///
/// \brief Angular constraint between two triangular faces
///
class DihedralConstraint : public PbdConstraint
{
public:
    ///
    /// \brief Constructor
    ///
    DihedralConstraint() : PbdConstraint(4) {}

    ///
    /// \brief Returns PBD constraint of type Type::Dihedral
    ///
    Type getType() const { return Type::Dihedral; }

    ///
    /// \brief initConstraint
    ///        p3
    ///       / | \
                ///      /  |  \
                ///     p0  |  p1
    ///      \  |  /
    ///       \ | /
    ///         p2
    /// \param model
    /// \param pIdx1 index of p0
    /// \param pIdx2 index of p1
    /// \param pIdx3 index of p2
    /// \param pIdx4 index of p3
    /// \param k stiffness
    ///
    void initConstraint(PositionBasedModel& model, const unsigned int& pIdx1, const unsigned int& pIdx2,
        const unsigned int& pIdx3, const unsigned int& pIdx4, const double k = 1e-3);

    ///
    /// \brief Solves the dihedral angular constraint
    ///
    bool solvePositionConstraint(PositionBasedModel &model);

public:
    double m_restAngle; ///> Rest angle
    double m_stiffness; ///> Angular stiffness
};
}

#endif // IMSTK_PBD_DIHEDRAL_CONSTRAINT_H