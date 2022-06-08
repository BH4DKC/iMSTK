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
#include "imstkPbdBaryPointToPointConstraint.h"
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

struct PenetrationData {

    // Triangle ID
    int triId;

    // Triangle verticese
    Vec3d* triVerts[3];
    Vec3i triVertIds;

    // Puncture barycentric coordinate on triangle
    Vec3d triBaryPuncturePoint;

};



///
/// \brief Surface collision disabled upon puncture
///
class NeedlePbdCH : public PbdCollisionHandling
{

protected:

    Vec3d m_needleDirection = Vec3d::Zero();

    // Flags for which entity is puncturing a triangle
    std::vector<bool> m_isNeedlePunctured;
    std::vector<bool> m_isThreadPunctured;

    // Vector of needle-triangle constraints (one sided, force triangle to follow needle)
    std::vector<std::shared_ptr<SurfaceInsertionConstraint>> pointTriangleConstraints;
    // Vector of thread-triangle constraints (one sided, force thread to follow triangle)
    std::vector<std::shared_ptr<PbdBaryPointToPointConstraint>> m_stitchConstraints;

    // Center of puncture points for stitching constraint
    Vec3d m_stitchCenter = Vec3d::Zero();


public:
    NeedlePbdCH() = default;
    ~NeedlePbdCH() override = default;

    IMSTK_TYPE_NAME(NeedlePbdCH)

    static Vec3d debugPt;
    static Vec3d debugPt2;

    // Create storage for penetration data for both the needle and the thread
    std::vector<PenetrationData> needlePData;
    std::vector<PenetrationData> threadPData;
    
    std::shared_ptr<PbdObject> m_threadObj;

    // Bool to activate stitching constraint
    bool m_stitch = false; 


    // Initialize interaction data
    void init(std::shared_ptr<PbdObject> threadObj) {
        
        // Setup pbd object
        auto pbdObj = std::dynamic_pointer_cast<PbdObject>(getInputObjectA());

        // Get surface mesh
        auto surfMesh = std::dynamic_pointer_cast<SurfaceMesh>(pbdObj->getCollidingGeometry());

       
        // Create storage for puncture states
        m_isNeedlePunctured.resize(surfMesh->getNumTriangles());
        m_isThreadPunctured.resize(surfMesh->getNumTriangles());
        
        // Initialize to false
        for (int triangleId = 0; triangleId < surfMesh->getNumTriangles(); triangleId++) {
            m_isNeedlePunctured[triangleId] = false;
            m_isThreadPunctured[triangleId] = false;
        }

        // Set thread object
        m_threadObj = threadObj;
    }

    
    // Create stitching constraints
    void Stitch() {

        // First, make sure at least 4 points have been penetrated by the thread
        if (threadPData.size() < 4) {
            std::cout << "Cant stitch less than 4 points" << std::endl;
            return;
        }
        
        if (threadPData.size() >= 4) {

            std::cout << "Stitching!" << std::endl;

            // Only calculate the center point once
            if (m_stitch == false) {

                // Calculate the average position of the points punctured by thread
                for (int pPointId = 0; pPointId < threadPData.size(); pPointId++) {

                    auto pPoint = (*threadPData[pPointId].triVerts[0]) * threadPData[pPointId].triBaryPuncturePoint[0]
                        + (*threadPData[pPointId].triVerts[1]) * threadPData[pPointId].triBaryPuncturePoint[1]
                        + (*threadPData[pPointId].triVerts[2]) * threadPData[pPointId].triBaryPuncturePoint[2];

                    for (int i = 0; i < 3; i++) {
                        m_stitchCenter[i] += pPoint[i] / (double)threadPData.size();
                    }
                }
            }
            
            m_stitch = true;

            // Create constraints to pull the puncture points to the center location
            for (int pPointId = 0; pPointId < threadPData.size(); pPointId++) {

                VertexMassPair ptA1;
                VertexMassPair ptA2;
                VertexMassPair ptA3;

                ptA1.vertex = threadPData[pPointId].triVerts[0];
                ptA2.vertex = threadPData[pPointId].triVerts[1];
                ptA3.vertex = threadPData[pPointId].triVerts[2];

                ptA1.invMass = 1.0; //  triangleInvMasses[threadPData[pPointId].triVertIds[0]];
                ptA2.invMass = 1.0; //  triangleInvMasses[threadPData[pPointId].triVertIds[0]];
                ptA3.invMass = 1.0; // triangleInvMasses[threadPData[pPointId].triVertIds[0]];
                
                Vec3d fakeVel = { 0.0, 0.0, 0.0 };
                ptA1.velocity = &fakeVel; //  &meshVelocity[threadPData[pPointId].triVertIds[0]];
                ptA2.velocity = &fakeVel; //  &meshVelocity[threadPData[pPointId].triVertIds[1]];
                ptA3.velocity = &fakeVel; //  &meshVelocity[threadPData[pPointId].triVertIds[2]];
                 
                std::vector<VertexMassPair> ptsA = { ptA1, ptA2, ptA3 };
                
                std::vector<double> weightsA = { threadPData[pPointId].triBaryPuncturePoint[0],
                    threadPData[pPointId].triBaryPuncturePoint[1],
                    threadPData[pPointId].triBaryPuncturePoint[2] };

                // Now create values for the central point
                VertexMassPair centerPt;

                centerPt.vertex = &m_stitchCenter;
                centerPt.invMass = 0.0; 

                Vec3d fakeVel2 = { 0.0, 0.0, 0.0 };
                centerPt.velocity = &fakeVel2;

                std::vector<VertexMassPair> ptsB = { centerPt };
                std::vector<double> weightsB = { 1.0 };

                auto constraint = std::make_shared<PbdBaryPointToPointConstraint>();
                constraint->initConstraint(
                    ptsA, weightsA,
                    ptsB, weightsB,
                    0.1, 0.0);

                // Add to list of constraints to be solved together
                m_stitchConstraints.push_back(constraint);
            }
        }
    }



    // The handle is called every timestep (update step)
    void handle(
        const std::vector<CollisionElement>& elementsA,
        const std::vector<CollisionElement>& elementsB) override
    {
        auto threadMesh = std::dynamic_pointer_cast<LineMesh>(m_threadObj->getCollidingGeometry());

        std::shared_ptr<VecDataArray<double, 3>> threadVerticesPtr = threadMesh->getVertexPositions();
        VecDataArray<double, 3>& threadVertices = *threadVerticesPtr;

        // InverseMasses for thread
        std::shared_ptr<PointSet> pointSetA = std::dynamic_pointer_cast<PointSet>(m_threadObj->getPhysicsGeometry());

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
        if (needleObj->getCollisionState() == NeedleObject::CollisionState::REMOVED || needleObj->getCollisionState() == NeedleObject::CollisionState::TOUCHING) {
            PbdCollisionHandling::handle(elementsA, elementsB); // (PBD Object, Needle Object)
        }

        // If inserted, find intersections and constrain to insertion points
        if (needleObj->getCollisionState() == NeedleObject::CollisionState::INSERTED) {
            
            // Scope for needle
            {
                // First, find new penetration points
                // Just use the tip of the needle (needle mesh is reversed)
                int tipSegmentId = needleMesh->getNumLines() - 1;

                Vec2i nodeIds = needleMesh->getLineIndices(tipSegmentId);
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

                    // If this triangle has not already been punctured
                    if (m_isNeedlePunctured[triangleId] == false) {

                        // Check for intersection
                        if (CollisionUtils::testSegmentTriangle(p, q, a, b, c, uvw) == true) {

                            m_isNeedlePunctured[triangleId] = true;

                            PenetrationData newPuncture;

                            newPuncture.triId = triangleId;

                            newPuncture.triVerts[0] = &a;
                            newPuncture.triVerts[1] = &b;
                            newPuncture.triVerts[2] = &c;

                            newPuncture.triVertIds[0] = physTriIds[0];
                            newPuncture.triVertIds[1] = physTriIds[1];
                            newPuncture.triVertIds[2] = physTriIds[2];

                            newPuncture.triBaryPuncturePoint = uvw;

                            // Create and save the puncture point
                            needlePData.push_back(newPuncture);
                            printf("Punctured triangle: %d \n", triangleId);

                        } // end if intersection
                    } // end if punctured
                } // end loop over triangles

                // Loop over penetration points and find nearest point on the needle
                // Note: Nearest point will likely be the point between two segments, its dualy defined
                for (int pPointId = 0; pPointId < needlePData.size(); pPointId++) {

                    // Start with large value
                    Vec3d closestPoint = { 1000 ,1000, 1000 };

                    Vec3d a = meshVertices[needlePData[pPointId].triVertIds[0]];
                    Vec3d b = meshVertices[needlePData[pPointId].triVertIds[1]];
                    Vec3d c = meshVertices[needlePData[pPointId].triVertIds[2]];

                    Vec3d baryPoint = needlePData[pPointId].triBaryPuncturePoint;
                    auto puncturePt = baryPoint[0] * a
                        + baryPoint[1] * b
                        + baryPoint[2] * c;

                    for (int segmentId = 0; segmentId < needleMesh->getNumLines(); segmentId++) {

                        Vec2i needleSegNodeIds = needleMesh->getLineIndices(segmentId);
                        const Vec3d x1 = needleVertices[needleSegNodeIds[0]];
                        const Vec3d x2 = needleVertices[needleSegNodeIds[1]];

                        int caseType = -1;

                        Vec3d segClosestPoint = CollisionUtils::closestPointOnSegment(puncturePt, x1, x2, caseType);

                        Vec3d newDist = segClosestPoint - puncturePt;
                        Vec3d oldDist = closestPoint - puncturePt;

                        if (newDist.norm() <= oldDist.norm()) {
                            closestPoint = segClosestPoint;
                        }

                    } // end loop over segments

                    // Check and see if the closest point is at the tips of the needle
                    // Note: Needle mesh is backwards
                    Vec3d diffTail = closestPoint - needleVertices[0];
                    Vec3d diffTip = closestPoint - needleVertices[needleMesh->getNumVertices() - 1];

                    if (diffTail.norm() < 1e-8 || diffTip.norm() < 1e-8) {

                        std::cout << "Unpuncture needle" << std::endl;

                        // If the tip of the needle has been removed, 
                        // this triangle is no longer punctured
                        if (diffTip.norm() < 1e-8) {
                            m_isNeedlePunctured[needlePData[pPointId].triId] = false;
                        }
                        needlePData.erase(needlePData.begin() + pPointId);
                        continue;
                    }

                    // Now that we have the closest point on the needle to this penetration point, we can
                    // generate and solve the constraint

                    auto pointTriangleConstraint = std::make_shared<SurfaceInsertionConstraint>();

                    VertexMassPair ptB1;
                    VertexMassPair ptB2;
                    VertexMassPair ptB3;

                    ptB1.vertex = &meshVertices[needlePData[pPointId].triVertIds[0]];
                    ptB2.vertex = &meshVertices[needlePData[pPointId].triVertIds[1]];
                    ptB3.vertex = &meshVertices[needlePData[pPointId].triVertIds[2]];

                    ptB1.invMass = triangleInvMasses[needlePData[pPointId].triVertIds[0]];
                    ptB2.invMass = triangleInvMasses[needlePData[pPointId].triVertIds[1]];
                    ptB3.invMass = triangleInvMasses[needlePData[pPointId].triVertIds[2]];

                    ptB1.velocity = &meshVelocity[needlePData[pPointId].triVertIds[0]];
                    ptB2.velocity = &meshVelocity[needlePData[pPointId].triVertIds[1]];
                    ptB3.velocity = &meshVelocity[needlePData[pPointId].triVertIds[2]];

                    pointTriangleConstraint->initConstraint(
                        needleRigid,
                        puncturePt, // Insertion Point
                        ptB1, ptB2, ptB3,
                        closestPoint,
                        baryPoint,
                        0.0, 1.0 // stiffness parameters
                    );

                    pointTriangleConstraint->solvePosition();

                    // std::cout << "Closest x = " << closestPoint[0] << std::endl;

                } // end loop over penetration points

            } // end scope for needle

            { // Scope for thread

            // Now on to the thread!!!
            // use the tip of the the same way as the tip of the needle 
            // to set up thread penetration points

            Vec3d threadTip = threadVertices[0];

            // First, find new penetration points

            Vec2i nodeIds = threadMesh->getLineIndices(0);
            const Vec3d p = threadVertices[nodeIds[0]];
            const Vec3d q = threadVertices[nodeIds[1]];

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

                // If this triangle has already been punctured
                if (m_isNeedlePunctured[triangleId] == true) {

                    // If it has not yet been punctured by thread
                    if (m_isThreadPunctured[triangleId] == false) {
                        
                        // Check for intersection
                        if (CollisionUtils::testSegmentTriangle(p, q, a, b, c, uvw) == true) {

                            std::cout << "Punctured with thread!" << std::endl;
                            m_isThreadPunctured[triangleId] = true;
                            
                            // Find matching needle puncture point
                            int needlePuncturePointId = -1;
                            for (int needlePPts = 0; needlePPts < needlePData.size(); needlePPts++) {

                                if (needlePData[needlePPts].triId == triangleId) {
                                    needlePuncturePointId = needlePPts;
                                }
                            }

                            std::cout << "Thread punctured triangle: " << needlePData[needlePuncturePointId].triId << std::endl;
                            std::cout << "Needle Puncture point " << needlePuncturePointId << std::endl;

                            // Create thread puncture point and copy in data from needle puncture point
                            PenetrationData newPuncture;

                            newPuncture.triId = needlePData[needlePuncturePointId].triId;
                            
                            newPuncture.triVerts[0] = &meshVertices[physTriIds[0]]; // needlePData[needlePuncturePointId].triVerts[0];
                            newPuncture.triVerts[1] = &meshVertices[physTriIds[0]]; // needlePData[needlePuncturePointId].triVerts[1];
                            newPuncture.triVerts[2] = &meshVertices[physTriIds[0]]; // needlePData[needlePuncturePointId].triVerts[2];

                            newPuncture.triVertIds[0] = physTriIds[0]; //  needlePData[needlePuncturePointId].triVertIds[0];
                            newPuncture.triVertIds[1] = physTriIds[1]; // needlePData[needlePuncturePointId].triVertIds[1];
                            newPuncture.triVertIds[2] = physTriIds[2]; //  needlePData[needlePuncturePointId].triVertIds[2];

                            newPuncture.triBaryPuncturePoint = uvw; // needlePData[needlePuncturePointId].triBaryPuncturePoint;

                            // Create and save the puncture point
                            threadPData.push_back(newPuncture);
                            printf("Punctured triangle with thread: %d \n", triangleId);

                        } // end if intersecting
                    } // end if punctured by thread
                } // end if punctured by needle
            } // end loop over surface mesh


            // Loop over penetration points and find nearest point on the thread
            // Note: Nearest point will likely be the point between two segments, its dualy defined
            for (int pPointId = 0; pPointId < threadPData.size(); pPointId++) {

                // Start with large value
                Vec3d closestPoint = { 1000 ,1000, 1000 };

                Vec3d a = meshVertices[threadPData[pPointId].triVertIds[0]];
                Vec3d b = meshVertices[threadPData[pPointId].triVertIds[1]];
                Vec3d c = meshVertices[threadPData[pPointId].triVertIds[2]];

                Vec3d baryPoint = threadPData[pPointId].triBaryPuncturePoint;
                
                auto puncturePt = baryPoint[0] * a
                    + baryPoint[1] * b
                    + baryPoint[2] * c;

                int closestSegment = -1;

                // Note: stopping before last segment for visualization
                for (int segmentId = 1; segmentId < threadMesh->getNumLines()-1; segmentId++) {

                    Vec2i threadSegNodeIds = threadMesh->getLineIndices(segmentId);
                    const Vec3d x1 = threadVertices[threadSegNodeIds[0]];
                    const Vec3d x2 = threadVertices[threadSegNodeIds[1]];

                    int caseType = -1;

                    Vec3d segClosestPoint = CollisionUtils::closestPointOnSegment(puncturePt, x1, x2, caseType);

                    Vec3d newDist = segClosestPoint - puncturePt;
                    Vec3d oldDist = closestPoint - puncturePt;

                    if (newDist.norm() <= oldDist.norm()) {
                        closestPoint = segClosestPoint;
                        closestSegment = segmentId;
                    }

                } // end loop over thread segments

                // Check and see if the closest point is at the tips of the thread
                Vec3d diffTip = closestPoint - threadVertices[0];
                Vec3d diffTail = closestPoint - threadVertices[threadMesh->getNumVertices() - 1];


                // Unpuncture if thread moves past last segment of the thread
                //if (diffTail.norm() < 1e-8 || diffTip.norm() < 1e-8) {

                //    std::cout << "Unpuncture thread" << std::endl;

                //    // If the tip of the needle has been removed, 
                //    // this triangle is no longer punctured
                //    if (diffTail.norm() < 1e-8) {
                //        m_isNeedlePunctured[threadPData[pPointId].triId] = false;
                //    }
                //    
                //    threadPData.erase(threadPData.begin() + pPointId);
                //    continue;
                //}

                // Now that we have the closest point on the needle to this penetration point, we can
                // generate and solve the constraint
                auto threadTriangleConstraint = std::make_shared<ThreadInsertionConstraint>();

                // Set of VM pairs for thread

                Vec2i nearestSegNodeIds = threadMesh->getLineIndices(closestSegment);
                const Vec3d p = threadVertices[nearestSegNodeIds[0]];
                const Vec3d q = threadVertices[nearestSegNodeIds[1]];


                VertexMassPair ptA1;
                ptA1.vertex = &threadVertices[nearestSegNodeIds[0]];
                ptA1.invMass = 1.0; // threadInvMasses[nearestSegNodeIds[0]];
                ptA1.velocity = &threadVelocity[nearestSegNodeIds[0]];

                VertexMassPair ptA2;
                ptA2.vertex = &threadVertices[nearestSegNodeIds[1]];
                ptA2.invMass = 1.0; // threadInvMasses[nearestSegNodeIds[1]];
                ptA2.velocity = &threadVelocity[nearestSegNodeIds[1]];

                // Thread barycentric intersection point
                Vec2d segBary = baryCentric(closestPoint, p, q);


                // Set of VM pairs for triangle
                VertexMassPair ptB1;
                VertexMassPair ptB2;
                VertexMassPair ptB3;

                ptB1.vertex = &meshVertices[threadPData[pPointId].triVertIds[0]];
                ptB2.vertex = &meshVertices[threadPData[pPointId].triVertIds[1]];
                ptB3.vertex = &meshVertices[threadPData[pPointId].triVertIds[2]];

                ptB1.invMass = triangleInvMasses[threadPData[pPointId].triVertIds[0]];
                ptB2.invMass = triangleInvMasses[threadPData[pPointId].triVertIds[0]];
                ptB3.invMass = triangleInvMasses[threadPData[pPointId].triVertIds[0]];

                ptB1.velocity = &meshVelocity[threadPData[pPointId].triVertIds[0]];
                ptB2.velocity = &meshVelocity[threadPData[pPointId].triVertIds[1]];
                ptB3.velocity = &meshVelocity[threadPData[pPointId].triVertIds[2]];

                threadTriangleConstraint->initConstraint(
                            ptA1, ptA2, segBary, 
                            ptB1, ptB2, ptB3, threadPData[pPointId].triBaryPuncturePoint,
                            0.95 , 0.0);

                threadTriangleConstraint->solvePosition();

            } // end loop over penetration points for thread

            } // end scope for thread

            // Solve stitching constraint
            if (m_stitch == true) {

                for (int i = 0; i < m_stitchConstraints.size(); i++) {

                    m_stitchConstraints[i]->solvePosition();
                }
            }
        } // end needle state puncture check
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
        double threshold = 0.82;

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
