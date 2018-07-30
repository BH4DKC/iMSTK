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

#ifndef NeedleTissueInteractionCH_h
#define NeedleTissueInteractionCH_h

// imstk
#include "imstkCollisionHandling.h"
#include "imstkFEMDeformableBodyModel.h"

namespace imstk
{
class CollidingObject;
class CollisionData;
class DeviceTracker;

///
/// \class NeedleTissueInteractionCH_h
///
/// \brief Implements needle-tissue interaction collision handling
///
class NeedleTissueInteraction : public CollisionHandling
{
public:

    ///
    /// \brief Constructor
    ///
    NeedleTissueInteraction(const Side& side,
        const CollisionData& colData,
        std::vector<LinearProjectionConstraint>* constraints, 
        std::shared_ptr<DeformableObject> femBody) :
        CollisionHandling(Type::BoneDrilling, side, colData), 
        m_projConstraints(constraints),
        m_deformableBody(femBody){}

    NeedleTissueInteraction() = delete;

    ///
    /// \brief Destructor
    ///
    ~NeedleTissueInteraction() = default;

    ///
    /// \brief Compute forces based on collision data
    ///
    void computeContactForces() override
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
    }

private:
    std::vector<LinearProjectionConstraint>* m_projConstraints;   ///> needle projection constraints
    std::shared_ptr<DeformableObject> m_deformableBody;
};
}

#endif // ifndef NeedleTissueInteractionCH_h