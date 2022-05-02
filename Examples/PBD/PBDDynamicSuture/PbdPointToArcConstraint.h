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
#include "imstkRigidObject2.h"
#include "imstkRbdConstraint.h"

using namespace imstk;

///
/// \class PbdPointToArcConstraint
///
/// \brief Constrains an point on a surface mesh to a rigid body arc needle
/// 
///
class PbdPointToArcConstraint : public PbdCollisionConstraint
{
public:

    // Needle
    std::shared_ptr<RigidBody> m_needle = nullptr;


    ///
    /// \param the Rigid body needle
    /// \param the center of the circle arc is defined with
    /// \param the radians/range of the arc, with relation to the arcBasis
    /// \param the radians/range of the arc, with relation to the arcBasis
    /// \param the radius of the circle the arc is defined with
    /// \param the basis of the arc. Where any point on the plane has a radian with relation to x,y columns.
    /// and the z column gives the normal of the plane the circle+arc lie on
    /// \param the fixed point
    ///
    PbdPointToArcConstraint(
        std::shared_ptr<RigidBody> needleObj,
        const Vec3d arcCenter, 
        const double beginRadian, 
        const double endRadian,
        const double arcCircleRadius, 
        const Mat3d arcBasis,
        const Vec3d contactPoint) :  PbdCollisionConstraint(1, 3), 
        m_arcCenter(arcCenter), m_arcBasis(arcBasis), m_arcCircleRadius(arcCircleRadius), m_beginRadian(beginRadian),
        m_endRadian(endRadian), m_contactPoint(contactPoint)
    {

    }

    ~PbdPointToArcConstraint() override = default;

public:

    
    bool computeValueAndGradient(
        double& c,
        std::vector<Vec3d>& dcdxA,
        std::vector<Vec3d>& dcdxB) const override
    {
        // Compute the direction and closest point to the arc from the Point
        const Vec3d circleDiff = m_contactPoint - m_arcCenter;
        const Vec3d dir = circleDiff.normalized();

        // m_arcBasis Should be orthonormal, this should project onto the axes
        const Mat3d  invArcBasis = m_arcBasis.transpose();
        const Vec3d  p   = invArcBasis * circleDiff;
        const double rad = atan2(-p[2], -p[0]) + PI;
            
        // Clamp to range (if closest point on circle is outside on range we want the end)
        const double clampedRad = std::min(std::max(rad, m_beginRadian), m_endRadian);
           
        // Finally compute the closest point to the arc using the new radian
        const Vec3d closestPt = 
            (cos(clampedRad) * m_arcBasis.col(0) + sin(clampedRad) * m_arcBasis.col(2)) * m_arcCircleRadius +
                                m_arcCenter;

        Vec3d diff = m_contactPoint - closestPt;
        diff = diff.normalized(); // gradient dcdx
        c = diff.norm();
        
        // Weight by berycentric coordinates?
        dcdxB[0] = diff;
        dcdxB[1] = diff;
        dcdxB[2] = diff;

        // Dont adjust position of needle, force mesh to follow needle
        dcdxA[0] = 0.0 * diff;

        return true;
    }

private:
    Vec3d  m_arcCenter       = Vec3d::Zero();
    Mat3d  m_arcBasis        = Mat3d::Zero(); // Should be orthonormal
    double m_arcCircleRadius = 0.0;
    double m_beginRadian     = 0.0;
    double m_endRadian       = 0.0;
    Vec3d m_contactPoint = Vec3d::Zero();

};