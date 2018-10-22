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
#include "imstkDeformableObject.h"

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
        std::shared_ptr<CollidingObject> needle,
        std::shared_ptr<DeformableObject> femBody) :
        CollisionHandling(Type::BoneDrilling, side, colData), 
        m_projConstraints(constraints),
        m_needle(needle),
        m_deformableBody(femBody){}

    NeedleTissueInteraction() = delete;

    ///
    /// \brief Destructor
    ///
    ~NeedleTissueInteraction() = default;

    ///
    /// \brief Set the scaling factor for the force
    ///
    void setScalingFactor(const double scaleFac) { m_scalingFactor = scaleFac; }

    ///
    /// \brief Compute forces based on collision data
    ///
    void computeContactForces() override;

    ///
    /// \brief Update the needle axis
    ///
    void updateNeedleAxis(const Vec3d& axis) { m_needleAxis = axis; m_needleAxis.normalize(); }

    ///
    /// \brief Set the motion state of the needle Rang: [0, 2]
    ///
    void setNeedleState(unsigned int needleState) { m_needleMotionState = needleState; };

    ///
    /// \brief Set the scaling factor for forces of insertion and retraction
    ///
    void setScalingFactorSliding(const double scale) { m_scalingFactorSliding = scale; };

private:
    std::vector<LinearProjectionConstraint>* m_projConstraints;   ///> needle projection constraints
    std::shared_ptr<DeformableObject> m_deformableBody;
    std::shared_ptr<CollidingObject> m_needle;
    Vec3d m_needleAxis;

    double m_scalingFactor = 1.0e-1;
    unsigned int m_needleMotionState = 0;
    double m_scalingFactorSliding = 7.0e-1; // Modify this to scale the forces using gray scale
};
}

#endif // ifndef NeedleTissueInteractionCH_h