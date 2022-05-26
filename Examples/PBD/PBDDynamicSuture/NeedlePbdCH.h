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

#include "imstkCollisionUtils.h"
#include "imstkLineMesh.h"
#include "imstkMacros.h"
#include "imstkPbdCollisionHandling.h"
#include "imstkPbdPointTriangleConstraint.h"
#include "imstkPointwiseMap.h"
#include "imstkRbdConstraint.h"
#include "imstkSurfaceMesh.h"
#include "imstkTetrahedralMesh.h"

#include "NeedleObject.h"
#include "SurfaceInsertionConstraint.h"



#include <iostream>
#include <math.h>  
#include <stdio.h>


using namespace imstk;

//struct InsertionData
//{
//    
//};
//
//InsertionData iData;


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

    static int counter;

    std::vector<Vec3d> m_penetrationPts;

    void init() {

        std::cout << "Inside init" << std::endl;
        // Setup pbd object
        auto pbdObj = std::dynamic_pointer_cast<PbdObject>(getInputObjectA());

        // Get surface mesh
        auto surfMesh = std::dynamic_pointer_cast<SurfaceMesh>(pbdObj->getCollidingGeometry());

        // Initialize collision state

        // Create storage for puncture points
        m_isPunctured.resize(surfMesh->getNumTriangles());
        
        // Initialize to false
        //for (int triangleId = 0; triangleId < surfMesh->getNumTriangles(); triangleId++) {
        //    m_isPunctured[triangleId] = false;
        //}

        // Create storage for penetration points
        m_penetrationPts.resize(surfMesh->getNumTriangles());

    }

protected:

    Vec3d m_collisionPt = Vec3d::Zero();
    Vec3d m_contactPt = Vec3d::Zero();
    Vec3d m_initAxes = Vec3d::Zero();
    Quatd m_initOrientation = Quatd::Identity();

    Vec3d m_needleDirection = Vec3d::Zero();
    Vec3d m_barycentricPoint = Vec3d::Zero();

    
    std::vector<bool> m_isPunctured;

    std::vector<Vec3d> m_insertionPoints; // puncture point on triangle
    std::vector<Vec3d> m_intersectionPoints; // Current intersection point
    std::vector<int> m_triIndices;
    std::vector<std::shared_ptr<SurfaceInsertionConstraint>> pointTriangleConstraints;

    // std::vector<SurfaceInsertionConstraint> pointTriangleConstraints;
    
    // Lazy approach, use std::tuple (check standard) with std::tie (break up struct into component parts with single statement)

   


    // The handle is called every timestep (update step)
    void handle(
        const std::vector<CollisionElement>& elementsA,
        const std::vector<CollisionElement>& elementsB) override
    {

        printf("Count = %d \n", counter++);

        // Setup needle and get collision state
        auto needleObj = std::dynamic_pointer_cast<NeedleObject>(getInputObjectB());
        auto needleMesh = std::dynamic_pointer_cast<LineMesh>(needleObj->getCollidingGeometry());
        

        // Setup pbd object
        auto pbdObj = std::dynamic_pointer_cast<PbdObject>(getInputObjectA());
        auto meshGeom = std::dynamic_pointer_cast<TetrahedralMesh>(pbdObj->getPhysicsGeometry());

        // Get surface mesh
        auto surfMesh = std::dynamic_pointer_cast<SurfaceMesh>(pbdObj->getCollidingGeometry());

        // Get vertex positions
        std::shared_ptr<VecDataArray<double, 3>> meshVerticesPtr = meshGeom->getVertexPositions();
        VecDataArray<double, 3>& meshVertices = *meshVerticesPtr;

        // For something to be a PbdObject it must have a pointset, it must also have invMasses defined
        std::shared_ptr<PointSet> pointSetA = std::dynamic_pointer_cast<PointSet>(pbdObj->getPhysicsGeometry());
        const std::shared_ptr<DataArray<double>> invMassesAPtr = std::dynamic_pointer_cast<DataArray<double>>(pointSetA->getVertexAttribute("InvMass"));
        const DataArray<double>& invMasses = *invMassesAPtr;

        // Warning, might return null ptr if type is wrong, cant deref nullpts, voodoo happens
        // auto invMasses = *std::dynamic_pointer_cast<DataArray<double>>(pointSetA->getVertexAttribute("InvMass"));
        // Do check on invMasses

        // Get velocities
        std::shared_ptr<VecDataArray<double, 3>> velocitiesAPtr = std::dynamic_pointer_cast<VecDataArray<double, 3>>(pointSetA->getVertexAttribute("Velocities"));
        VecDataArray<double, 3>& meshVelocity = *velocitiesAPtr;

        // Get rigid object needle
        auto needleRigid = std::dynamic_pointer_cast<RigidBody>(getInputObjectB());

        // This will give the vertex position of the ith vertex
        // needleMesh->getVertexPosition(i)

        std::shared_ptr<VecDataArray<double, 3>> needleVerticesPtr = needleMesh->getVertexPositions();
        VecDataArray<double, 3>& needleVertices = *needleVerticesPtr;

        m_needleDirection = (needleVertices[35] - needleVertices[34]).normalized();

        auto one2one = std::static_pointer_cast<PointwiseMap>(pbdObj->getPhysicsToCollidingMap());

        Vec3d insertionVector;
        


        // Do it the normal way if not inserted
        if (elementsB.size() == 0) needleObj->setCollisionState(NeedleObject::CollisionState::REMOVED);
        
        NeedleObject::CollisionState state = needleObj->getCollisionState();
        
        if (state == NeedleObject::CollisionState::REMOVED) {
            PbdCollisionHandling::handle(elementsA, elementsB); // (PBD Object, Needle Object)

            // WARNING: PATCH, needs to be reworked
            /*for (int triangleId = 0; triangleId < surfMesh->getNumTriangles(); triangleId++) {
                m_isPunctured[triangleId] = false;
            }*/
        }

        // If inserted, find intersections and constrain to insertion points
        if (state == NeedleObject::CollisionState::INSERTED) {
            
            int numIntersections = 0;
            
            for (int segmentId = 0; segmentId < needleMesh->getNumLines(); segmentId++) {

                Vec2i nodeIds = needleMesh->getLineIndices(segmentId);
                const Vec3d p = needleVertices[nodeIds[0]];
                const Vec3d q = needleVertices[nodeIds[1]];

                // Loop over all triangles in the surface mesh
                for (int triangleId = 0; triangleId < surfMesh->getNumTriangles(); triangleId++) {

                    auto surfTriIds = surfMesh->getTriangleIndices(triangleId);

                    // Indices of the vertices on the physics mesh
                    Vec3i physTriIds;
                    physTriIds[0] = one2one->getParentVertexId(surfTriIds[0]);
                    physTriIds[1] = one2one->getParentVertexId(surfTriIds[1]);
                    physTriIds[2] = one2one->getParentVertexId(surfTriIds[2]);

                    Vec3d a = meshVertices[physTriIds[0]];
                    Vec3d b = meshVertices[physTriIds[1]];
                    Vec3d c = meshVertices[physTriIds[2]];

                    // Barycentric coordinates of interseciton point
                    Vec3d uvw = Vec3d::Zero();
                    
                    // Check for intersection
                    if (CollisionUtils::testSegmentTriangle(p, q, a, b, c, uvw) == true){
                        
                        numIntersections++;

                        
                        // if this triangle has not been inserted before, add to list and create constraint
                        if (m_isPunctured[triangleId] == false) {
                            
                            // std::cout << "New Point!" << std::endl;
                            std::cout << "Punctured triangle: " << triangleId << std::endl;
                            
                            // Save first penetration point 
                            m_penetrationPts[triangleId] = uvw;

                            // Change state to inserted
                            m_isPunctured[triangleId] = true;
                    
                        }

                        // Show current insertion point on screen
                        // debugPt = uvw[0] * a + uvw[1] * b + uvw[2] * c;

                        // Show initial insertion point
                        /*debugPt2 = m_penetrationPts[triangleId][0] * a
                            + m_penetrationPts[triangleId][1] * b
                            + m_penetrationPts[triangleId][2] * c;*/

                        // Cut here
                        /*Vec3d contactPt = uvw[0] * a + uvw[1] * b + uvw[2] * c;
                        Vec3d needleVelocity = Vec3d::Zero();

                        VertexMassPair vmNeedle;
                        vmNeedle.vertex = &contactPt;
                        vmNeedle.invMass = 1.0;
                        vmNeedle.velocity = &needleVelocity;

                        const std::vector<VertexMassPair> bodiesFirst = { vmNeedle };

                        VertexMassPair ptB1;
                        VertexMassPair ptB2;
                        VertexMassPair ptB3;

                        ptB1.vertex = &meshVertices[physTriIds[0]];
                        ptB2.vertex = &meshVertices[physTriIds[1]];
                        ptB3.vertex = &meshVertices[physTriIds[2]];

                        ptB1.invMass = invMasses[physTriIds[0]];
                        ptB2.invMass = invMasses[physTriIds[1]];
                        ptB3.invMass = invMasses[physTriIds[2]];

                        ptB1.velocity = &meshVelocity[physTriIds[0]];
                        ptB2.velocity = &meshVelocity[physTriIds[1]];
                        ptB3.velocity = &meshVelocity[physTriIds[2]];
                        
                        auto intersectionPt = uvw[0] * a + uvw[1] * b + uvw[2] * c;

                        const std::vector<VertexMassPair> bodiesSecond = { ptB1, ptB2, ptB3 };

                        auto pointTriangleConstraint = std::make_shared<SurfaceInsertionConstraint>();

                        auto puncturePt = m_penetrationPts[triangleId][0] * a
                            + m_penetrationPts[triangleId][1] * b
                            + m_penetrationPts[triangleId][2] * c;*/

                        //pointTriangleConstraint->initConstraint(
                        //    needleRigid,
                        //    puncturePt, // Insertion Point
                        //    ptB1, ptB2, ptB3,
                        //    intersectionPt,
                        //    m_penetrationPts[triangleId],
                        //    0.0, 1.0 // stiffness parameters
                        //);

                        // pointTriangleConstraint->solvePosition();
                        
                    } // end intersection test
                } // end loop over triangles

            } // end loop over line segments in needle

            // Possibly print out changing geometry

            // redo sweep and check for unpuncture
            for (int segmentId = 0; segmentId < needleMesh->getNumLines(); segmentId++) {

                Vec2i nodeIds = needleMesh->getLineIndices(segmentId);
                const Vec3d p = needleVertices[nodeIds[0]];
                const Vec3d q = needleVertices[nodeIds[1]];

                // Loop over all triangles in the surface mesh
                for (int triangleId = 0; triangleId < surfMesh->getNumTriangles(); triangleId++) {

                    if (m_isPunctured[triangleId] == true) {
                        
                        auto surfTriIds = surfMesh->getTriangleIndices(triangleId);

                        // Indices of the vertices on the physics mesh
                        Vec3i physTriIds;
                        physTriIds[0] = one2one->getParentVertexId(surfTriIds[0]);
                        physTriIds[1] = one2one->getParentVertexId(surfTriIds[1]);
                        physTriIds[2] = one2one->getParentVertexId(surfTriIds[2]);

                        Vec3d a = meshVertices[physTriIds[0]];
                        Vec3d b = meshVertices[physTriIds[1]];
                        Vec3d c = meshVertices[physTriIds[2]];

                        // Barycentric coordinates of interseciton point
                        Vec3d uvw = Vec3d::Zero();

                        // Check for intersection
                        bool test = CollisionUtils::testSegmentTriangle(p, q, a, b, c, uvw);

                        if (test == false) {
 
                            std::cout << "Unpunctured triangle: " << triangleId << std::endl;
                            m_isPunctured[triangleId] = false;

                        }
                    } // end if punctured
                } // end loop over triangles
            } // end loop over needle segments
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

        // Arbitrary threshold
        double threshold = 0.9;

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
                    needleObj->setPrevCollisionState(NeedleObject::PrevCollisionState::INSERTED);
                }
            }
        }

        if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {
            PbdCollisionHandling::addVTConstraint(ptA, ptB1, ptB2, ptB3, stiffnessA, stiffnessB);
        }
    }
};


/*                      Vec3d contactPt = uvw[0] * a + uvw[1] * b + uvw[2] * c;
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

                        ptB1.vertex = &meshVertices[physTriIds[0]];
                        ptB2.vertex = &meshVertices[physTriIds[1]];
                        ptB3.vertex = &meshVertices[physTriIds[2]];

                        ptB1.invMass = invMasses[physTriIds[0]];
                        ptB2.invMass = invMasses[physTriIds[1]];
                        ptB3.invMass = invMasses[physTriIds[2]];

                        ptB1.velocity = &meshVelocity[physTriIds[0]];
                        ptB2.velocity = &meshVelocity[physTriIds[1]];
                        ptB3.velocity = &meshVelocity[physTriIds[2]];


                        auto worldCoords = m_penetrationPts[triangleId][0] * a
                            + m_penetrationPts[triangleId][1] * b
                            + m_penetrationPts[triangleId][2] * c;

                        debugPt2 = worldCoords;*/


                        // const std::vector<VertexMassPair> bodiesSecond = { ptB1, ptB2, ptB3 };

                        //auto pointTriangleConstraint = std::make_shared<SurfaceInsertionConstraint>();

                        //pointTriangleConstraint->initConstraint(
                        //    needleRigid,
                        //    worldCoords, // Insertion Point
                        //    ptB1, ptB2, ptB3,
                        //    contactPt,
                        //    m_barycentricPoint,
                        //    0.0, 1.0 // stiffness parameters
                        //);

                        // pointTriangleConstraint->solvePosition();

                        //pointTriangleConstraints.push_back(pointTriangleConstraint);

                        //// Consider moving this to the back, after the sweep

                        //pointTriangleConstraints.back()->initConstraint(
                        //    needleRigid,
                        //    worldCoords, // Insertion Point
                        //    ptB1, ptB2, ptB3, 
                        //    contactPt, 
                        //    m_barycentricPoint,
                        //    0.0, 1.0 // stiffness parameters
                        //);

                        // Possibly move outside of loop
                        // pointTriangleConstraints.back()->solvePosition();

                        // Also need to test for removal.
                        // for all triangles in inserted state, sweep over needle segments to test for removal