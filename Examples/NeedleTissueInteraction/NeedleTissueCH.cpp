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

#include "NeedleTissueCH.h"
#include "imstkTetrahedralMesh.h"

namespace imstk
{

void
NeedleTissueInteraction::computeContactForces()
{
    // clear constraints from previous frame
    m_projConstraints->clear();

    // return if no collisions are detected in the current frame
    if (m_colData.NeedleColData.size() == 0)
    {
        return;
    }

    // set up displacements for next timestep
    auto dynaModel = m_deformableBody->getDynamicalModel();
    auto uPrev = m_deformableBody->getDisplacements();
    auto vPrev = m_deformableBody->getVelocities();

    const auto physicsTetMesh = std::dynamic_pointer_cast<TetrahedralMesh>(m_deformableBody->getPhysicsGeometry());
    const auto nodePositions = physicsTetMesh->getVertexPositions();
    const auto dt = dynaModel->getTimeStep();

    // set up constraints
    for (auto& colData : m_colData.NeedleColData)
    {
        auto _3i = colData.nodeId * 3;
        Vec3d uPrevNode(uPrev(_3i), uPrev(_3i + 1), uPrev(_3i + 2));
        Vec3d vPrevNode(vPrev(_3i), vPrev(_3i + 1), vPrev(_3i + 2));
        auto deltaV = (colData.pointOnNeedle - physicsTetMesh->getInitialVertexPosition(colData.nodeId) - uPrevNode) / dt - vPrevNode;
        auto deltaVProj = deltaV - deltaV.dot(colData.axis)*colData.axis;

        LinearProjectionConstraint s(colData.nodeId, true);
        s.setProjectionToLine(colData.nodeId, colData.axis);
        s.setValue(deltaVProj);

        m_projConstraints->push_back(s);
    }

    // compute forces
    Vec3d force(0., 0., 0.);
    Vec3d forceSliding(0., 0., 0.);
    for (auto& colData : m_colData.NeedleColData)
    {
        auto nodalDisp = physicsTetMesh->getVertexPosition(colData.nodeId) -
            physicsTetMesh->getInitialVertexPosition(colData.nodeId);

        force += -nodalDisp*m_scalingFactor;

        forceSliding -= (colData.pointOnNeedle - colData.prevPos)*m_scalingFactorSliding;
    }
    force += forceSliding;

    // Update object contact force
    m_needle->appendForce(force);
}

}