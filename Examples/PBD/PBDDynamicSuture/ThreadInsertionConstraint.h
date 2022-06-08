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
/// \class ThreadInsertionConstraint
///
/// \brief Constrains an point on a line mesh to a PBD surface mesh
/// 
///
class ThreadInsertionConstraint : public PbdCollisionConstraint
{


private:


public:


    Vec2d m_threadBaryPt;
    Vec3d m_triangleBaryPt;

    Vec3d m_triInsertionPoint;
    Vec3d m_threadInsertionPoint;

    ///
    /// \param 
    /// \param  
    ///
    ThreadInsertionConstraint() :  PbdCollisionConstraint(2, 3) // (num thread verts, num triangle verts)
    {

    }

    ~ThreadInsertionConstraint() override = default;

public:


    void initConstraint(
        VertexMassPair ptA1,
        VertexMassPair ptA2,
        Vec2d threadBaryPoint,
        VertexMassPair ptB1,
        VertexMassPair ptB2,
        VertexMassPair ptB3,
        Vec3d triBaryPoint, 
        double stiffnessA,
        double stiffnessB) {

        // Vertex mass pairs for thread
        m_bodiesFirst[0] = ptA1;
        m_bodiesFirst[1] = ptA2;

        // Barycentric coordinate on thread of intersection point
        m_threadBaryPt = threadBaryPoint;

        // Computing world coordinates of intersecting point along thread
        m_threadInsertionPoint = m_threadBaryPt[0] * (*m_bodiesFirst[0].vertex)
            + m_threadBaryPt[1] * (*m_bodiesFirst[1].vertex);

        // Vertex mass pairs for triangle
        m_bodiesSecond[0] = ptB1;
        m_bodiesSecond[1] = ptB2;
        m_bodiesSecond[2] = ptB3;

        // Barycentric coordinate of puncture point on triangle
        m_triangleBaryPt = triBaryPoint;

        // Computing world coordinates of puncture point
        m_triInsertionPoint = m_triangleBaryPt[0] * (*m_bodiesSecond[0].vertex)
            + m_triangleBaryPt[1] * (*m_bodiesSecond[1].vertex)
            + m_triangleBaryPt[2] * (*m_bodiesSecond[2].vertex);

        // Saving stiffness
        m_stiffnessA = stiffnessA;
        m_stiffnessB = stiffnessB;

    }
    
    bool computeValueAndGradient(
        double& c,
        std::vector<Vec3d>& dcdxA,
        std::vector<Vec3d>& dcdxB) const override
    {

        // Move thread such that the thread stays intersected with the 
        // puncture point on the triangle

        Vec3d diff = m_triInsertionPoint-m_threadInsertionPoint;  // gradient dcdx m_threadInsertionPoint - m_triInsertionPoint
        
        c = 0.5*diff.norm();

        diff = diff.normalized(); // gradient dcdx

        // Move thread to follow insertion point
        dcdxA[0] = diff * m_threadBaryPt[0];
        dcdxA[1] = diff * m_threadBaryPt[1];


        // Move triangle to follow thread point (WARNING: Currently inactive)
        dcdxB[0] = 0.0*diff * m_triangleBaryPt[0];
        dcdxB[1] = 0.0*diff * m_triangleBaryPt[1];
        dcdxB[2] = 0.0*diff * m_triangleBaryPt[2];
        
        return true;
    }
};