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
    Vec3d m_initContactPt = Vec3d::Zero();
    Vec3d m_initAxes = Vec3d::Zero();
    Quatd m_initOrientation = Quatd::Identity();

    // The handle is called every timestep
    void handle(
        const std::vector<CollisionElement>& elementsA,
        const std::vector<CollisionElement>& elementsB) override
    {
        // Do it the normal way
        PbdCollisionHandling::handle(elementsA, elementsB); // (PBD Object, Needle Object)

  
        auto needleObj = std::dynamic_pointer_cast<NeedleObject>(getInputObjectB());
        NeedleObject::CollisionState state = needleObj->getCollisionState();

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

                // Constrain along the axes, whilst allowing "pushing" of the contact point
                auto pointToArcConstraint = std::make_shared<PbdPointToArcConstraint>(
                    needleObj->getRigidBody(),
                    arcCenter,
                    arcBeginRad,
                    arcEndRad,
                    arcRadius,
                    arcBasis,
                    m_collisionPt); // needs to be contact point
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
        m_collisionPt = *ptA.vertex;

        auto needleObj = std::dynamic_pointer_cast<NeedleObject>(getInputObjectB());

        // If removed and we are here, we must now be touching
        if (needleObj->getCollisionState() == NeedleObject::CollisionState::REMOVED)
        {
            needleObj->setCollisionState(NeedleObject::CollisionState::TOUCHING);
        }

        // If touching we may test for insertion

        // Where can I get this contact normal?  Is it saved somewhere, or do I need to calculate it?
        // const Vec3d n = contactNormal.normalized();
        if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {
            // Get all inwards force
            // const double fN = std::max(-contactNormal.dot(needleObj->getRigidBody()->getForce()), 0.0);
            const double fN = 6.0;
            // If the normal force exceeds threshold the needle may insert
            if (fN > needleObj->getForceThreshold())
            {
                LOG(INFO) << "Puncture!";
                needleObj->setCollisionState(NeedleObject::CollisionState::INSERTED);

                //// Record the initial contact point
                //m_initOrientation = Quatd(needleObj->getCollidingGeometry()->getRotation());
                //
                //// This needs to be the barycentric point on the triangle in contact
                //// m_initContactPt = contactPt;
            }
        }

        if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {
            PbdCollisionHandling::addVTConstraint(ptA, ptB1, ptB2, ptB3, stiffnessA, stiffnessB); 
        }
    }

};