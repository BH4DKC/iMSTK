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
/// \class SurfaceInsertionConstraint
///
/// \brief Constrains an point on a surface mesh to a rigid body arc needle
/// 
///
class SurfaceInsertionConstraint : public PbdCollisionConstraint
{


private:


public:

    // Needle
    std::shared_ptr<RigidBody> m_needle = nullptr;
    Vec3d m_insertionPoint;
    Vec3d m_barycentricPt;
    VertexMassPair m_ptB1;
    VertexMassPair m_ptB2;
    VertexMassPair m_ptB3;
    Vec3d m_punctureVector;
    Vec3d m_contactPt;

    ///
    /// \param the Rigid body needle
    /// \param  
    ///
    SurfaceInsertionConstraint() :  PbdCollisionConstraint(1, 3)
    {

    }

    ~SurfaceInsertionConstraint() override = default;

public:

    void initConstraint(
        std::shared_ptr<RigidBody> needle,
        Vec3d insertionPoint,
        VertexMassPair ptB1, 
        VertexMassPair ptB2,
        VertexMassPair ptB3, 
        Vec3d contactPt,
        Vec3d barycentricPt,
        double stiffnessA,
        double stiffnessB) {

        m_needle = needle;
        m_insertionPoint = insertionPoint;
        m_ptB1 = ptB1;
        m_ptB2 = ptB2;
        m_ptB3 = ptB3;
        // m_punctureVector = punctureVector;
        m_contactPt = contactPt;

        m_barycentricPt = barycentricPt;

        m_bodiesSecond[0] = ptB1;
        m_bodiesSecond[1] = ptB2;
        m_bodiesSecond[2] = ptB3;

        m_stiffnessA = stiffnessA;
        m_stiffnessB = stiffnessB;

    }
    
    bool computeValueAndGradient(
        double& c,
        std::vector<Vec3d>& dcdxA,
        std::vector<Vec3d>& dcdxB) const override
    {

        // Get current position of puncture point
        // Move triangle to match motion of needle


        Vec3d diff = m_contactPt - m_insertionPoint;
        // Vec3d diff = m_insertionPoint - m_contactPt;
        // diff = diff.normalized(); // gradient dcdx
        c = diff.norm();

        diff = diff.normalized(); // gradient dcdx
        // Weight by berycentric coordinates
        dcdxB[0] = diff * m_barycentricPt[0];
        dcdxB[1] = diff * m_barycentricPt[1];
        dcdxB[2] = diff * m_barycentricPt[2];

        // Dont adjust position of needle, force mesh to follow needle
        // But maybe do though
        dcdxA[0] = 0.0*diff;

        return true;
    }
};