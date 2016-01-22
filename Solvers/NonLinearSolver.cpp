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

#include "Solvers/NonLinearSolver.h"

NonLinearSolver::NonLinearSolver():
    sigma(std::array<double, 2> {.1, .5}),
    alpha(1e-4),
    armijoMax(30)
{
    this->updateIterate = [](const core::Vectord &dx, core::Vectord &x)
    {
        x += dx;
    };
}

//---------------------------------------------------------------------------
double NonLinearSolver::
armijo(const core::Vectord &dx, core::Vectord &x, const double previousFnorm)
{
    /// Temporaries used in the line search
    std::array<double, 3> fnormSqr  = {previousFnorm*previousFnorm, 0.0, 0.0};
    std::array<double, 3> lambda    = {this->sigma[0]*this->sigma[1], 1.0, 1.0};

    /// Initialize temporaries
    double currentFnorm = this->nonLinearSystem->eval(x).norm();

    // Exit if the function norm satisfies the Armijo-Goldstein condition
    if(currentFnorm < (1.0 - this->alpha * lambda[0])*previousFnorm)
    {
        return currentFnorm;
    }

    // Save iterate in case this fails
    auto x_old = x;

    // Starts Armijo line search loop
    size_t i;
    for(i = 0; i < this->armijoMax; ++i)
    {
        /// Update x and keep books on lambda
        this->updateIterate(-lambda[0]*dx,x);
        lambda[2] = lambda[1];
        lambda[1] = lambda[0];

        currentFnorm = this->nonLinearSystem->eval(x).norm();

        // Exit if the function norm satisfies the Armijo-Goldstein condition
        if(currentFnorm < (1.0 - this->alpha * lambda[0])*previousFnorm)
        {
            return currentFnorm;
        }

        /// Update function norms
        fnormSqr[2] = fnormSqr[1];
        fnormSqr[1] = currentFnorm * currentFnorm;

        /// Apply the three point parabolic model
        this->parabolicModel(fnormSqr, lambda);
    }

    if(i == this->armijoMax)
    {
        // TODO: Add to logger
//         std::cout << "Maximum number of Armijo iterations reached." << std::endl;
    }


    return currentFnorm;
}

//---------------------------------------------------------------------------
void NonLinearSolver::
parabolicModel(const std::array<double,3> &fnorm, std::array<double,3> &lambda)
{
    /// Compute the coefficients for the interpolation polynomial:
    ///     p(lambda) = fnorm[0] + (b*lambda + a*lambda^2)/d1, where
    ///         d1 = (lambda[1] - lambda[2])*lambda[1]*lambda[2] < 0
    ///     if a > 0, then we have a concave up curvature and lambda defaults to:
    ///         lambda = sigma[0]*lambda
    double a1 = lambda[2] * (fnorm[1] - fnorm[0]);
    double a2 = lambda[1] * (fnorm[2] - fnorm[0]);
    double a = a1 - a2;

    if(a >= 0)
    {
        lambda[0] = this->sigma[0] * lambda[1];
        return;
    }

    double b = lambda[1] * a2 - lambda[2] * a1;
    double newLambda = -.5 * b / a;

    if(newLambda < this->sigma[0] * lambda[1])
    {
        newLambda = this->sigma[0] * lambda[1];
    }

    if(newLambda > this->sigma[1] * lambda[1])
    {
        newLambda = this->sigma[1] * lambda[1];
    }

    lambda[0] = newLambda;
}

//---------------------------------------------------------------------------
void NonLinearSolver::setSigma(const std::array<double,2> &newSigma)
{
    this->sigma = newSigma;
}

//---------------------------------------------------------------------------
const std::array<double,2> &NonLinearSolver::getSigma() const
{
    return this->sigma;
}

//---------------------------------------------------------------------------
void NonLinearSolver::setAlpha(const double newAlpha)
{
    this->alpha = newAlpha;
}

//---------------------------------------------------------------------------
double NonLinearSolver::getAlpha() const
{
    return this->alpha;
}

//---------------------------------------------------------------------------
void NonLinearSolver::setArmijoMax(const size_t newArmijoMax)
{
    this->armijoMax = newArmijoMax;
}

//---------------------------------------------------------------------------
size_t NonLinearSolver::getArmijoMax() const
{
    return this->armijoMax;
}

//---------------------------------------------------------------------------
void NonLinearSolver::setSystem(std::shared_ptr< SystemOfEquations > newSystem)
{
    this->nonLinearSystem = newSystem;
}

//---------------------------------------------------------------------------
std::shared_ptr< SystemOfEquations > NonLinearSolver::getSystem() const
{
    return this->nonLinearSystem;
}

//---------------------------------------------------------------------------
void NonLinearSolver::setSystem(const NonLinearSolver::FunctionType &F)
{
    this->nonLinearSystem = std::make_shared<SystemOfEquations>();
    this->nonLinearSystem->setFunction(F);
}

//---------------------------------------------------------------------------
void NonLinearSolver::
setUpdateIterate(const NonLinearSolver::UpdateIterateType& newUpdateIterate)
{
    this->updateIterate = newUpdateIterate;
}