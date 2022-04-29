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

#include "imstkPbdCollisionConstraint.h"
#include "imstkCollisionHandling.h"
#include "imstkSurfaceMesh.h"
#include "imstkMacros.h"

using namespace imstk;

///
/// \class RbdPointToArcConstraint
///
/// \brief Constrains an rigid body arc geometry to a point by computing the
/// linear force and angular torque to get the arc to the point
///
class EmbeddedConstraint : public PbdCollisionConstraint
{
public:


    EmbeddedConstraint()
        : PbdConstraint()
    {
        // Check orthonormal basis
    }

    ~EmbeddedConstraint() override = default;

public:
    void computeValueAndGradient(
        const VecDataArray<double, 3>& currVertexPositions,
        double& c,
        std::vector<Vec3d>& dcdx)


private:
    Vec3d  m_arcCenter = Vec3d::Zero();
    Mat3d  m_arcBasis = Mat3d::Zero(); // Should be orthonormal
    double m_arcCircleRadius = 0.0;
    double m_beginRadian = 0.0;
    double m_endRadian = 0.0;

    Vec3d m_fixedPoint = Vec3d::Zero();

    double m_beta = 0.05;
};