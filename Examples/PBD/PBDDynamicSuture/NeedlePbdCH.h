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

#include "imstkMacros.h"
#include "imstkPbdCollisionHandling.h"

#include "NeedleObject.h"
#include "PbdPointToArcConstraint.h"
#include "imstkRbdConstraint.h"
#include "imstkSurfaceMesh.h"
#include "imstkLineMesh.h"

#include <iostream>
#include <math.h>  


using namespace imstk;

///
/// \brief Surface collision disabled upon puncture
///
class NeedlePbdCH : public PbdCollisionHandling
{
public:
    NeedlePbdCH() = default;
    ~NeedlePbdCH() override = default;

    IMSTK_TYPE_NAME(NeedlePbdCH)

    static Vec3d debugPt;


protected:
  
    Vec3d m_collisionPt;
    Vec3d m_contactPt = Vec3d::Zero();
    Vec3d m_initAxes = Vec3d::Zero();
    Quatd m_initOrientation = Quatd::Identity();

    Vec3d m_needleDirection = Vec3d::Zero();
    Vec3d m_barycentricPoint = Vec3d::Zero();


    // The handle is called every timestep
    void handle(
        const std::vector<CollisionElement>& elementsA,
        const std::vector<CollisionElement>& elementsB) override
    {

        // Do it the normal way
        PbdCollisionHandling::handle(elementsA, elementsB); // (PBD Object, Needle Object)


        // Setup pbd object
        auto pbdObj = std::dynamic_pointer_cast<PbdObject>(getInputObjectA());
        auto meshGeom = std::dynamic_pointer_cast<SurfaceMesh>(pbdObj->getCollidingGeometry());


        // Setup needle and get collision state
        auto needleObj = std::dynamic_pointer_cast<NeedleObject>(getInputObjectB());
        auto needleMesh = std::dynamic_pointer_cast<LineMesh>(needleObj->getCollidingGeometry());
        NeedleObject::CollisionState state = needleObj->getCollisionState();

        std::shared_ptr<VecDataArray<double, 3>> needleVerticesPtr = needleMesh->getVertexPositions();
        VecDataArray<double, 3>& needleVertices = *needleVerticesPtr;

        m_needleDirection = (needleVertices[0] - needleVertices[1]).normalized();
        //std::cout << m_needleDirection[0] << std::endl;

        // If the collision elements exists, check state
        if (elementsB.size() != 0){

            //moveToState(X)
            //handleState(X)

            if (state == NeedleObject::CollisionState::INSERTED)
            {
                LOG(INFO) << "Inside!";
                
                const Mat3d& arcBasis = needleObj->getArcBasis();
                const Vec3d& arcCenter = needleObj->getArcCenter();
                const double arcRadius = needleObj->getArcRadius();
                const double arcBeginRad = needleObj->getBeginRad();
                const double arcEndRad = needleObj->getEndRad();

                //// Constrain along the axes, whilst allowing "pushing" of the contact point
                //auto pointToArcConstraint = std::make_shared<PbdPointToArcConstraint>(
                //    needleObj->getRigidBody(),
                //    arcCenter,
                //    arcBeginRad,
                //    arcEndRad,
                //    arcRadius,
                //    arcBasis,
                //    m_collisionPt); // needs to be contact point
            }
        }
    }

    ///
    /// \brief Add a vertex-triangle constraint
    ///
    void addVTConstraint(
        VertexMassPair ptA,
        VertexMassPair ptB1, VertexMassPair ptB2, VertexMassPair ptB3,
        double stiffnessA, double stiffnessB) override
    {
        debugPt = *ptA.vertex;

        // This is actually the point on the tip of the needle, not on the triangle, 
        // it needs to be associated with the triangle
        m_collisionPt = *ptA.vertex;

       

        // std::cout << m_barycentricPoint[0] << std::endl;

        auto needleObj = std::dynamic_pointer_cast<NeedleObject>(getInputObjectB());

        // If removed and we are here, we must now be touching
        if (needleObj->getCollisionState() == NeedleObject::CollisionState::REMOVED)
        {
            needleObj->setCollisionState(NeedleObject::CollisionState::TOUCHING);

            // Calculate the barycentric coordinates
            Vec3d p = *ptA.vertex;
            Vec3d a = *ptB1.vertex;
            Vec3d b = *ptB2.vertex;
            Vec3d c = *ptB3.vertex;

            m_barycentricPoint = baryCentric(p, a, b, c);
        }

        // If touching we may test for insertion
        // Calculate the surface normal using the set of verticese associated with the triangle
        // Calculate a vector using the first two points of the line mesh needle
        // Normalize both and use dot product to project onto each other, if close to 1 asume its inserted
        // Possibly add constact time or pseudo force calculation to know if penetration occurs
        
        Vec3d surfNormal = Vec3d::Zero();


        // Assuming traingle has points a,b,c
        // NOTE: How do I access this data? I need to get it from the collision data somehow
        Vec3d ab; 
        Vec3d ac;

        // Calculate surface normal
        surfNormal = (ab.cross(ac)).normalized();

        // Get vector pointing in direction of needle
        
        // Use absolute value to ignore direction issues
        auto dotProduct = fabs(m_needleDirection.dot(surfNormal));

        // Arbitrary threshold
        double threshold = 0.8;

        if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {

            // If the needle is close to perpindicular to the face if may insert
            // Note: This is a short term solution
            if (dotProduct > threshold)
            {
                LOG(INFO) << "Puncture!";
                needleObj->setCollisionState(NeedleObject::CollisionState::INSERTED);

                // Record the initial contact point
                // NOTE: This needs to be the barycentric point on the triangle in contact
                // m_contactPt = contactPt;
            }
        }

        if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {
            PbdCollisionHandling::addVTConstraint(ptA, ptB1, ptB2, ptB3, stiffnessA, stiffnessB); 
        }
    }
};