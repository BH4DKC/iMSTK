// This file is part of the SimMedTK project.
// Copyright (c) Center for Modeling, Simulation, and Imaging in Medicine,
//                        Rensselaer Polytechnic Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//---------------------------------------------------------------------------
//
// Authors:
//
// Contact:
//---------------------------------------------------------------------------

#include "Solvers/ForwardGaussSeidel.h"

ForwardGaussSeidel::ForwardGaussSeidel(
    const core::SparseMatrixd &A,
    const core::Vectord &rhs):
    U(A.triangularView<Eigen::StrictlyUpper>()),
    L(A.triangularView<Eigen::Lower>())
{
    this->linearSystem = std::make_shared<LinearSystem<core::SparseMatrixd>>(A, rhs);
}

//---------------------------------------------------------------------------
void ForwardGaussSeidel::iterate(core::Vectord &x, bool updateResidual)
{
    x = this->linearSystem->getRHSVector() - U * x;
    L.solveInPlace(x);

    if (updateResidual)
    {
        this->linearSystem->computeResidual(x, this->residual);
    }
}