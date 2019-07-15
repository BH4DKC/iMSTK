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

#include "imstkPbdFEMConstraint.h"

namespace imstk
{
///
/// \class FEMTetConstraint
///
/// \brief The FEMTetConstraint class class for constraint as the elastic energy
/// computed by linear shape functions with tetrahedral mesh.
///
class PbdFEMTetConstraint : public PbdFEMConstraint
{
public:
    ///
    /// \brief Constructor
    ///
    explicit PbdFEMTetConstraint(MaterialType mtype = MaterialType::StVK) :
        PbdFEMConstraint(4, mtype) {}

    ///
    /// \brief Get the type of FEM constraint
    ///
    inline Type getType() const override { return Type::FEMTet; }

    ///
    /// \brief Initialize the tetrahedral FEM constraint
    ///
    bool initConstraint(PbdModel& model,
                        const size_t& pIdx1, const size_t& pIdx2,
                        const size_t& pIdx3, const size_t& pIdx4);

    ///
    /// \brief Solve the tetrahedral FEM constraint
    ///
    bool solvePositionConstraint(PbdModel& model) override;
};
} // imstk
