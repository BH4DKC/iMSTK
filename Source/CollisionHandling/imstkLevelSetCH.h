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

#include "imstkCollisionHandling.h"
#include "imstkMacros.h"

#include <unordered_set>
#include <vector>

namespace imstk
{
class CollisionData;
class LevelSetDeformableObject;
class RigidObject2;

///
/// \class LevelSetCH
///
/// \brief Applies impulses to the leveset given point direction collision data
/// propotional to the force on the rigid object
///
class LevelSetCH : public CollisionHandling
{
public:
    LevelSetCH();
    virtual ~LevelSetCH() override;

    IMSTK_TYPE_NAME(LevelSetCH)

public:
    void setInputLvlSetObj(std::shared_ptr<LevelSetDeformableObject> lvlSetObj);
    void setInputRigidObj(std::shared_ptr<RigidObject2> rbdObj);

    std::shared_ptr<LevelSetDeformableObject> getLvlSetObj();
    std::shared_ptr<RigidObject2> getRigidObj();

public:
    ///
    /// \brief Adds point to the mask allowing it to apply an impulse to the levelset
    ///
    void addPoint(int id) { m_ptIdMask.insert(id); }

    ///
    /// \brief Allow all points to effect the levelset
    ///
    void maskAllPoints();

    ///
    /// \brief Unmask all points
    ///
    void unmaskAllPoints() { m_ptIdMask.clear(); }

    ///
    /// \brief Set/Get Scale of the velocity used for the levelset, default 0.1
    ///
    double getLevelSetVelocityScaling() const { return m_velocityScaling; }
    void setLevelSetVelocityScaling(const double velocityScaling) { m_velocityScaling = velocityScaling; }

    ///
    /// \brief Set/Get whether the velocity used on the levelset should be proportional to the force of the rigid body
    /// along the normal of the levelset
    ///
    void setUseProportionalVelocity(const bool useProportionalForce) { m_useProportionalForce = useProportionalForce; }
    bool getUseProportionalVelocity() const { return m_useProportionalForce; }

    ///
    /// \brief Set/Get the size + sigma of gaussian kernel used to apply impulse in levelset
    ///
    void setKernel(const int size, const double sigma = 1.0);
    int getKernelSize() const { return m_kernelSize; }
    double getKernelSigma() const { return m_kernelSigma; }

protected:
    ///
    /// \brief Compute forces and velocities based on collision data
    ///
    void handle(
        const std::vector<CollisionElement>& elementsA,
        const std::vector<CollisionElement>& elementsB) override;

protected:
    std::unordered_set<int> m_ptIdMask;
    double  m_velocityScaling      = 0.1;
    bool    m_useProportionalForce = false;
    int     m_kernelSize    = 3;
    double  m_kernelSigma   = 1.0;
    double* m_kernelWeights = nullptr;
};
} // namespace imstk