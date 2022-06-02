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
#include "imstkMath.h"
#include "imstkPbdCollisionHandling.h"
#include "imstkPbdPointTriangleConstraint.h"
#include "imstkPointwiseMap.h"
#include "imstkRbdConstraint.h"
#include "imstkSurfaceMesh.h"
#include "imstkTetrahedralMesh.h"

#include "NeedleObject.h"
#include "SurfaceInsertionConstraint.h"
#include "ThreadInsertionConstraint.h"


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

struct PenetrationData {

    // Triangle ID
    int triId;

    // Triangle verticese
    Vec3d* triVerts[3];
    Vec3i triVertIds;

    // Puncture barycentric coordinate on triangle
    Vec3d triBaryPuncturePoint;

    // Possibly dont add these here, compute at runtime
    // Segment Vertices
    // Vec2d* segVerts[2];

    // Segment barycentric insertion point
    // Vec2d segBaryPuncturePoint;

};



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
    std::vector<PenetrationData> pData;
    std::shared_ptr<PbdObject> m_threadObj;

    void init(std::shared_ptr<PbdObject> threadObj) {

        std::cout << "Inside init NeedlePbdCH" << std::endl;
        // Setup pbd object
        auto pbdObj = std::dynamic_pointer_cast<PbdObject>(getInputObjectA());

        // Get surface mesh
        auto surfMesh = std::dynamic_pointer_cast<SurfaceMesh>(pbdObj->getCollidingGeometry());

        // Initialize collision state

        // Create storage for puncture points
        m_isPunctured.resize(surfMesh->getNumTriangles());
        
        // Initialize to false
        for (int triangleId = 0; triangleId < surfMesh->getNumTriangles(); triangleId++) {
            m_isPunctured[triangleId] = false;
        }

        // Create storage for penetration points
        m_penetrationPts.resize(surfMesh->getNumTriangles());

        // Initialize penetration data
        pData.push_back(PenetrationData());
        pData[0].triId = -1;

        // Set thread object
        m_threadObj = threadObj;

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

        int count = 0;
        count++;

        // printf("Count = %d \n", counter++);

        auto threadMesh = std::dynamic_pointer_cast<LineMesh>(m_threadObj->getCollidingGeometry());

        std::shared_ptr<VecDataArray<double, 3>> threadVerticesPtr = threadMesh->getVertexPositions();
        VecDataArray<double, 3>& threadVertices = *threadVerticesPtr;

        // InverseMasses for thread
        std::shared_ptr<PointSet> pointSetA = std::dynamic_pointer_cast<PointSet>(m_threadObj->getPhysicsGeometry());
        const std::shared_ptr<DataArray<double>> threadInvMassesAPtr = std::dynamic_pointer_cast<DataArray<double>>(pointSetA->getVertexAttribute("InvMass"));
        const DataArray<double>& threadInvMasses = *threadInvMassesAPtr;

        // Get velocities
        std::shared_ptr<VecDataArray<double, 3>> threadVelocitiesAPtr = std::dynamic_pointer_cast<VecDataArray<double, 3>>(pointSetA->getVertexAttribute("Velocities"));
        VecDataArray<double, 3>& threadVelocity = *threadVelocitiesAPtr;


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
        std::shared_ptr<PointSet> pointSetB = std::dynamic_pointer_cast<PointSet>(pbdObj->getPhysicsGeometry());
        const std::shared_ptr<DataArray<double>> triangleInvMassesAPtr = std::dynamic_pointer_cast<DataArray<double>>(pointSetB->getVertexAttribute("InvMass"));
        const DataArray<double>& triangleInvMasses = *triangleInvMassesAPtr;

        // Warning, might return null ptr if type is wrong, cant deref nullpts, voodoo happens
        // auto invMasses = *std::dynamic_pointer_cast<DataArray<double>>(pointSetA->getVertexAttribute("InvMass"));
        // Do check on invMasses

        // Get velocities
        std::shared_ptr<VecDataArray<double, 3>> velocitiesAPtr = std::dynamic_pointer_cast<VecDataArray<double, 3>>(pointSetB->getVertexAttribute("Velocities"));
        VecDataArray<double, 3>& meshVelocity = *velocitiesAPtr;

        // Get rigid object needle
        auto needleRigid = std::dynamic_pointer_cast<RigidBody>(getInputObjectB());

        // This will give the vertex position of the ith vertex
        // needleMesh->getVertexPosition(i)

        std::shared_ptr<VecDataArray<double, 3>> needleVerticesPtr = needleMesh->getVertexPositions();
        VecDataArray<double, 3>& needleVertices = *needleVerticesPtr;

        m_needleDirection = (needleVertices[35] - needleVertices[34]).normalized();

        auto one2one = std::static_pointer_cast<PointwiseMap>(pbdObj->getPhysicsToCollidingMap());

        // Do it the normal way if not inserted
        if (elementsB.size() == 0) {
            needleObj->setCollisionState(NeedleObject::CollisionState::REMOVED);
            // printf("Needle entirely removed in element size test\n");
        }

        // NeedleObject::CollisionState state = needleObj->getCollisionState();
        if (needleObj->getCollisionState() == NeedleObject::CollisionState::REMOVED || needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING) {
            PbdCollisionHandling::handle(elementsA, elementsB); // (PBD Object, Needle Object)
        }

        // PbdCollisionHandling::handle(elementsA, elementsB); // (PBD Object, Needle Object)

        // If inserted, find intersections and constrain to insertion points
        if (needleObj->getCollisionState() == NeedleObject::CollisionState::INSERTED) {
            
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
                        
                            printf("Punctured triangle:  %d \n", triangleId);
                            
                            // Save first penetration point 
                            m_penetrationPts[triangleId] = uvw;

                            // Change state to inserted
                            m_isPunctured[triangleId] = true;

                            if (pData[pData.size()-1].triId != triangleId) {
                                
                                
                                pData[pData.size() - 1].triId = triangleId;

                                pData[pData.size() - 1].triBaryPuncturePoint = uvw;

                                pData[pData.size() - 1].triVerts[0] = &meshVertices[physTriIds[0]];
                                pData[pData.size() - 1].triVerts[1] = &meshVertices[physTriIds[1]];
                                pData[pData.size() - 1].triVerts[2] = &meshVertices[physTriIds[2]];

                                pData[pData.size() - 1].triVertIds[0] = physTriIds[0];
                                pData[pData.size() - 1].triVertIds[1] = physTriIds[1];
                                pData[pData.size() - 1].triVertIds[2] = physTriIds[2];

                                pData.push_back(PenetrationData());
                                std::cout << "Penetration data lenght = " << pData.size() << std::endl;
                                std::cout << "Penetrated triangle = " << pData[0].triId << std::endl;
                                // printf("Penetration data lenght = %d /n", (int)pData.size());

                            }
                        }

                        Vec3d contactPt = uvw[0] * a + uvw[1] * b + uvw[2] * c;
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

                        ptB1.invMass = triangleInvMasses[physTriIds[0]];
                        ptB2.invMass = triangleInvMasses[physTriIds[1]];
                        ptB3.invMass = triangleInvMasses[physTriIds[2]];

                        ptB1.velocity = &meshVelocity[physTriIds[0]];
                        ptB2.velocity = &meshVelocity[physTriIds[1]];
                        ptB3.velocity = &meshVelocity[physTriIds[2]];
                        
                        auto intersectionPt = uvw[0] * a + uvw[1] * b + uvw[2] * c;

                        const std::vector<VertexMassPair> bodiesSecond = { ptB1, ptB2, ptB3 };

                        auto pointTriangleConstraint = std::make_shared<SurfaceInsertionConstraint>();

                        auto puncturePt = m_penetrationPts[triangleId][0] * a
                            + m_penetrationPts[triangleId][1] * b
                            + m_penetrationPts[triangleId][2] * c;

                        pointTriangleConstraint->initConstraint(
                            needleRigid,
                            puncturePt, // Insertion Point
                            ptB1, ptB2, ptB3,
                            intersectionPt,
                            m_penetrationPts[triangleId],
                            0.0, 1.0 // stiffness parameters
                        );

                        pointTriangleConstraint->solvePosition();
                        
                    } // end intersection test
                } // end loop over triangles
            } // end loop over line segments in needle

            // Loop over all triangles in the surface mesh to check for unpuncture
            
            bool needleInsertionCheck = false;
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

                    bool segmentInsertionCheck = false;

                    // check that this traingle is still punctured
                    for (int segmentId = 0; segmentId < needleMesh->getNumLines(); segmentId++) {

                        Vec2i nodeIds = needleMesh->getLineIndices(segmentId);
                        const Vec3d p = needleVertices[nodeIds[0]];
                        const Vec3d q = needleVertices[nodeIds[1]];
                        
                        // Barycentric coordinates of interseciton point
                        Vec3d uvw = Vec3d::Zero();

                        // Check for intersection
                        if (CollisionUtils::testSegmentTriangle(p, q, a, b, c, uvw)) {

                            segmentInsertionCheck = true;
                            needleInsertionCheck = true;
                            // break;
                        }

                    } // end loop over segments

                    if (segmentInsertionCheck == false) {
                       
                        printf("Unpunctured triangle: %d \n", triangleId);
                        m_isPunctured[triangleId] = false;
                    }
                } // end if punctured
            } // end loop over triangles

            // If no intersections, then change needle state 
            if (needleInsertionCheck == false) {
                needleObj->setCollisionState(NeedleObject::CollisionState::REMOVED);
                printf("Needle entirely removed in test unpuncture\n");
            }


            // if(count%100 == 0) printf("Penetration data lenght = %d /n", (int)pData.size());
        } // end needle state puncture check

        // Check for insertion with thread and generate constraints if so
        // printf("Num thread segments : %d \n", threadMesh->getNumLines());
        if (needleObj->getPrevCollisionState() == NeedleObject::PrevCollisionState::INSERTED) {
            
            // printf("Num thread segments : %d \n", threadMesh->getNumLines());
            for (int segmentId = 1; segmentId < threadMesh->getNumLines(); segmentId++) {

                // Test for insertion with triangle punctured by needle
                Vec2i nodeIds = threadMesh->getLineIndices(segmentId);
                auto p = &threadVertices[nodeIds[0]];
                auto q = &threadVertices[nodeIds[1]];

                // debugPt2 = threadVertices[1];

                Vec3d a = *pData[0].triVerts[0];
                Vec3d b = *pData[0].triVerts[1];
                Vec3d c = *pData[0].triVerts[2];
                

                Vec3d uvw = Vec3d::Zero();

                if (CollisionUtils::testSegmentTriangle(*p, *q, a, b, c, uvw)) {

                    auto intersectionPt = uvw[0] * a + uvw[1] * b + uvw[2] * c;

                    auto threadTriangleConstraint = std::make_shared<ThreadInsertionConstraint>();

                    // Set of VM pairs for thread
                    VertexMassPair ptA1;
                    ptA1.vertex = &threadVertices[nodeIds[0]];
                    ptA1.invMass = threadInvMasses[nodeIds[0]];
                    ptA1.velocity = &threadVelocity[nodeIds[0]];


                    VertexMassPair ptA2;
                    ptA2.vertex = &threadVertices[nodeIds[1]];
                    ptA2.invMass = threadInvMasses[nodeIds[1]];
                    ptA2.velocity = &threadVelocity[nodeIds[1]];

                    // Thread barycentric intersection point
                    Vec2d segBary = baryCentric(intersectionPt, *p, *q);
                    // std::cout << "Thread barycentric pt first index : " << segBary[0] << std::endl;

                    // Set of VM pairs for triangle
                    VertexMassPair ptB1;
                    VertexMassPair ptB2;
                    VertexMassPair ptB3;

                    ptB1.vertex = pData[0].triVerts[0];
                    ptB2.vertex = pData[0].triVerts[1];
                    ptB3.vertex = pData[0].triVerts[2];

                    ptB1.invMass = 1.0;
                    ptB2.invMass = 1.0;
                    ptB3.invMass = 1.0;

                    ptB1.velocity = &meshVelocity[pData[0].triVertIds[0]];
                    ptB2.velocity = &meshVelocity[pData[0].triVertIds[1]];
                    ptB3.velocity = &meshVelocity[pData[0].triVertIds[2]];


                    threadTriangleConstraint->initConstraint(
                        ptA1, ptA2, segBary, 
                        ptB1, ptB2, ptB3, pData[0].triBaryPuncturePoint,
                        1.0, 1.0);

                    threadTriangleConstraint->solvePosition();

                    debugPt = segBary[0]* (*p) + segBary[1]* (*q); // green debug point
                    
                   /* debugPt2 = pData[0].triBaryPuncturePoint[0] * a
                    + pData[0].triBaryPuncturePoint[1] * b
                    + pData[0].triBaryPuncturePoint[2] * c;*/

                    // printf("Thread Inserted!!! \n");  
                } // end if intersection
            } // end loop over segments in thread
        } // end if inserted 
         

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
            printf("Touching!\n");
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
                needleObj->setPrevCollisionState(NeedleObject::PrevCollisionState::INSERTED);
            }
        }

        if (needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING)
        {
            // printf("Addint VT constraint\n");
            PbdCollisionHandling::addVTConstraint(ptA, ptB1, ptB2, ptB3, stiffnessA, stiffnessB);
        }
    }
};

