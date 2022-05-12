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
#include "imstkPbdPointTriangleConstraint.h"
#include "imstkRbdConstraint.h"
#include "imstkSurfaceMesh.h"
#include "imstkPointwiseMap.h"
#include "imstkTetrahedralMesh.h"
#include "imstkLineMesh.h"
#include "imstkCollisionUtils.h"

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
    static Vec3d debugPt2;

protected:
  
    Vec3d m_collisionPt = Vec3d::Zero();
    Vec3d m_contactPt = Vec3d::Zero();
    Vec3d m_initAxes = Vec3d::Zero();
    Quatd m_initOrientation = Quatd::Identity();

    Vec3d m_needleDirection = Vec3d::Zero();
    Vec3d m_barycentricPoint = Vec3d::Zero();


    // The handle is called every timestep (update step)
    void handle(
        const std::vector<CollisionElement>& elementsA,
        const std::vector<CollisionElement>& elementsB) override
    {

        // Setup needle and get collision state
        auto needleObj = std::dynamic_pointer_cast<NeedleObject>(getInputObjectB());
        auto needleMesh = std::dynamic_pointer_cast<LineMesh>(needleObj->getCollidingGeometry());
        NeedleObject::CollisionState state = needleObj->getCollisionState();

        // Do it the normal way
        PbdCollisionHandling::handle(elementsA, elementsB); // (PBD Object, Needle Object)

        //if (elementsB.size() != 0) needleObj->setCollisionState(NeedleObject::CollisionState::REMOVED);
        //if (state == NeedleObject::CollisionState::REMOVED) {
        //    PbdCollisionHandling::handle(elementsA, elementsB); // (PBD Object, Needle Object)
        //}
        // NOTE: Add check for mapping

        // Setup pbd object
        auto pbdObj = std::dynamic_pointer_cast<PbdObject>(getInputObjectA());
        auto meshGeom = std::dynamic_pointer_cast<TetrahedralMesh>(pbdObj->getPhysicsGeometry());

        std::shared_ptr<VecDataArray<double, 3>> meshVerticesPtr = meshGeom->getVertexPositions();
        VecDataArray<double, 3>& meshVertices = *meshVerticesPtr;

        // For something to be a PbdObject it must have a pointset, it must also have invMasses defined
        std::shared_ptr<PointSet> pointSetA = std::dynamic_pointer_cast<PointSet>(pbdObj->getPhysicsGeometry());
        
        const std::shared_ptr<DataArray<double>> invMassesAPtr = std::dynamic_pointer_cast<DataArray<double>>(pointSetA->getVertexAttribute("InvMass"));
        const DataArray<double>& invMasses = *invMassesAPtr;
        
        // Warning, might return null ptr if type is wrong, cant deref nullpts, voodoo happens
        // auto invMasses = *std::dynamic_pointer_cast<DataArray<double>>(pointSetA->getVertexAttribute("InvMass"));
        // Do check on invMasses

        std::shared_ptr<VecDataArray<double, 3>> velocitiesAPtr = std::dynamic_pointer_cast<VecDataArray<double, 3>>(pointSetA->getVertexAttribute("Velocities"));
        VecDataArray<double, 3>& meshVelocity = *velocitiesAPtr;


        

        // Get rigid object needle
        auto needleRigid = std::dynamic_pointer_cast<RigidBody>(getInputObjectB());

        // This will give the vertex position of the ith vertex
        // needleMesh->getVertexPosition(i)

        std::shared_ptr<VecDataArray<double, 3>> needleVerticesPtr = needleMesh->getVertexPositions();
        VecDataArray<double, 3>& needleVertices = *needleVerticesPtr;

        m_needleDirection = (needleVertices[35] - needleVertices[34]).normalized();
        
        // If the collision elements exists, check state
        if (elementsB.size() != 0){

            if (state == NeedleObject::CollisionState::INSERTED)
            {

                const Mat3d& arcBasis = needleObj->getArcBasis();
                const Vec3d& arcCenter = needleObj->getArcCenter();
                const double arcRadius = needleObj->getArcRadius();
                const double arcBeginRad = needleObj->getBeginRad();
                const double arcEndRad = needleObj->getEndRad();

                const CollisionElement& contactTriangle = elementsA[0];

                // Add CHECK statement for type
                // Need grasping if block


                // CollisionElement is a union, must check types
                auto cellId = contactTriangle.m_element.m_CellIndexElement;

                // Need one to one map
                auto one2one = std::static_pointer_cast<PointwiseMap>(pbdObj->getPhysicsToCollidingMap());

                Vec3i triIds;
                triIds[0] = one2one->getParentVertexId(cellId.ids[0]);
                triIds[1] = one2one->getParentVertexId(cellId.ids[1]);
                triIds[2] = one2one->getParentVertexId(cellId.ids[2]);

               /* triIds[0] = cellId.ids[0];
                triIds[1] = cellId.ids[1];
                triIds[2] = cellId.ids[2];*/


                Vec3d a = meshVertices[triIds[0]];
                Vec3d b = meshVertices[triIds[1]];
                Vec3d c = meshVertices[triIds[2]];

                auto worldCoords = m_barycentricPoint[0] * a
                    + m_barycentricPoint[1] * b
                    + m_barycentricPoint[2] * c;

                debugPt2 = worldCoords;


                // Find inserted segment
                for (int segmentId = 0; segmentId < needleMesh->getNumLines(); segmentId++) {

                    Vec2i nodeIds = needleMesh->getLineIndices(segmentId);

                    const Vec3d p = needleVertices[nodeIds[0]];
                    const Vec3d q = needleVertices[nodeIds[1]];

                    Vec3d insertionVector = Vec3d::Zero();
                    Vec3d uvw = Vec3d::Zero();

                    // If this segment is inserted constraint the surface mesh insertion
                    // point to track the new collision point
                    if (CollisionUtils::testSegmentTriangle(p, q, a, b, c, uvw)) 
                    {
                        debugPt = uvw[0]*a  + uvw[1]*b + uvw[2]*c;
                        Vec3d contactPt = uvw[0] * a + uvw[1] * b + uvw[2] * c;
                        insertionVector = (p - q).normalized();

                        Vec3d needleVelocity = Vec3d::Zero();

                        VertexMassPair vmNeedle;
                        vmNeedle.vertex = &contactPt;
                        vmNeedle.invMass = 1.0;
                        vmNeedle.velocity = &needleVelocity;

                        const std::vector<VertexMassPair> bodiesFirst = { vmNeedle };

                        VertexMassPair ptB1;
                        VertexMassPair ptB2;
                        VertexMassPair ptB3;

                        ptB1.vertex = &meshVertices[triIds[0]];
                        ptB2.vertex = &meshVertices[triIds[1]];
                        ptB3.vertex = &meshVertices[triIds[2]];

                        ptB1.invMass = invMasses[triIds[0]];
                        ptB2.invMass = invMasses[triIds[1]];
                        ptB3.invMass = invMasses[triIds[2]];

                        ptB1.velocity = &meshVelocity[triIds[0]];
                        ptB2.velocity = &meshVelocity[triIds[1]];
                        ptB3.velocity = &meshVelocity[triIds[2]];

                        const std::vector<VertexMassPair> bodiesSecond = { ptB1, ptB2, ptB3 };
                        auto pointTriangleConstraint = std::make_shared<PbdPointToArcConstraint>();
                        pointTriangleConstraint->initConstraint(
                            needleRigid,
                            worldCoords, // Insertion Point
                            ptB1, ptB2, ptB3, 
                            contactPt, 
                            m_barycentricPoint,
                            0.0, 1.0 // stiffness parameters
                        );

                        // TRY:  Clearing elements array if insertion is happening

                        pointTriangleConstraint->solvePosition();

                        //// BS
                        auto correctionVec = contactPt - worldCoords;

                    }
                }
            }
        }
        else {
            needleObj->setCollisionState(NeedleObject::CollisionState::REMOVED);
            needleObj->setPrevCollisionState(NeedleObject::PrevCollisionState::REMOVED);

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
        auto needleObj = std::dynamic_pointer_cast<NeedleObject>(getInputObjectB());

        // If removed and we are here, we must now be touching
        if (needleObj->getCollisionState() == NeedleObject::CollisionState::REMOVED)
        {
            needleObj->setCollisionState(NeedleObject::CollisionState::TOUCHING);
        }

        // If touching we may test for insertion
        // Calculate the surface normal using the set of verticese associated with the triangle and normalize
        // use dot product to project onto the needle stabing direction, if close to 1 assume its inserted
        // Possibly add contact time or pseudo force calculation to know if penetration occurs
        
        Vec3d surfNormal = Vec3d::Zero();

        // Note, assumes closed mesh

        // Assuming traingle has points a,b,c
        Vec3d ab = *ptB2.vertex - *ptB1.vertex;
        Vec3d ac = *ptB3.vertex - *ptB1.vertex;

        // Calculate surface normal
        surfNormal = (ac.cross(ab)).normalized();

        // Get vector pointing in direction of needle
        
        // Use absolute value to ignore direction issues
        auto dotProduct = fabs(m_needleDirection.dot(surfNormal));

        // std::cout << dotProduct << std::endl;

        // Arbitrary threshold
        double threshold = 0.9;

        Vec3d worldCoords = Vec3d::Zero();

        // Calculate the barycentric coordinates
        Vec3d p = *ptA.vertex;
        Vec3d a = *ptB1.vertex;
        Vec3d b = *ptB2.vertex;
        Vec3d c = *ptB3.vertex;

      /*  m_barycentricPoint = baryCentric(p, a, b, c);

        worldCoords = 
            m_barycentricPoint[0] * a
            + m_barycentricPoint[1] * b
            + m_barycentricPoint[2] * c;*/

        // debugPt = worldCoords;

        if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {

            // If the needle is close to perpindicular to the face if may insert
            // Note: This is a short term solution
            if (dotProduct > threshold)
            {
                LOG(INFO) << "Puncture!";
                needleObj->setCollisionState(NeedleObject::CollisionState::INSERTED);


                if (needleObj->getPrevCollisionState() == NeedleObject::PrevCollisionState::REMOVED)
                {
                    // Calculate the barycentric coordinates
                    Vec3d p = *ptA.vertex; // NOTE: Probably underneath the triangle, verify
                    Vec3d a = *ptB1.vertex;
                    Vec3d b = *ptB2.vertex;
                    Vec3d c = *ptB3.vertex;

                    m_barycentricPoint = baryCentric(p, a, b, c);

                    std::cout << "TEST" << std::endl;

                    needleObj->setPrevCollisionState(NeedleObject::PrevCollisionState::INSERTED);
                
                }

                //// Calculate the barycentric coordinates
                //Vec3d p = *ptA.vertex;
                //Vec3d a = *ptB1.vertex;
                //Vec3d b = *ptB2.vertex;
                //Vec3d c = *ptB3.vertex;

                //m_barycentricPoint = baryCentric(p, a, b, c);

               

                // Also stored punctured triangle global cell index of the punctured triangle
                // save vertex indices?

                // Record the initial contact point
                // NOTE: This needs to be the barycentric point on the triangle in contact
                // m_contactPt = contactPt;
                // m_contactPt = worldCoords;
            }
        }

        if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {
            PbdCollisionHandling::addVTConstraint(ptA, ptB1, ptB2, ptB3, stiffnessA, stiffnessB);
        }
    }
};