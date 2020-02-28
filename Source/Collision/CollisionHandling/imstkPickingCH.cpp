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

#include "imstkPickingCH.h"
#include "imstkDeformableObject.h"
#include "imstkParallelUtils.h"

namespace imstk
{
PickingCH::PickingCH(const CollisionHandling::Side&       side,
                     const std::shared_ptr<CollisionData> colData,
                     std::shared_ptr<CollidingObject>     obj) :
    CollisionHandling(Type::NodalPicking, side, colData),
    m_object(std::dynamic_pointer_cast<DeformableObject>(obj))
{
}

void
PickingCH::processCollisionData()
{
    if (m_object)
    {
        this->addPickConstraints(m_object);
    }
    else
    {
        LOG(FATAL) << "PickingCH::handleCollision error: "
                   << "no picking collision handling available the object";
    }
}

void
PickingCH::addPickConstraints(std::shared_ptr<DeformableObject> deformableObj)
{
    m_DynamicLinearProjConstraints->clear();

    if (m_colData->NodePickData.isEmpty())
    {
        return;
    }

    if (deformableObj == nullptr)
    {
        LOG(FATAL) << "PenaltyRigidCH::addPickConstraints error: "
                   << " not a deformable object.";
        return;
    }

    const auto& Uprev = deformableObj->getDisplacements();
    const auto& Vprev = deformableObj->getVelocities();

    auto PhysTetMesh = std::dynamic_pointer_cast<PointSet>(deformableObj->getPhysicsGeometry());
    auto dT          = std::dynamic_pointer_cast<FEMDeformableBodyModel>(m_object->getDynamicalModel())->getTimeIntegrator()->getTimestepSize();

    // If collision data, append LPC constraints
    ParallelUtils::SpinLock lock;
    ParallelUtils::parallelFor(m_colData->NodePickData.getSize(),
        [&](const size_t idx) {
            const auto& cd     = m_colData->NodePickData[idx];
            const auto nodeDof = static_cast<Eigen::Index>(3 * cd.nodeIdx);
            const auto vprev   = Vec3d(Vprev(nodeDof), Vprev(nodeDof + 1), Vprev(nodeDof + 2));
            const auto uprev   = Vec3d(Uprev(nodeDof), Uprev(nodeDof + 1), Uprev(nodeDof + 2));
            const auto x       = (cd.ptPos + PhysTetMesh->getVertexPosition(cd.nodeIdx) -
                                  PhysTetMesh->getInitialVertexPosition(cd.nodeIdx) - uprev) / dT - vprev;

            auto pickProjector = LinearProjectionConstraint(cd.nodeIdx, true);
            pickProjector.setProjectorToDirichlet(static_cast<unsigned int>(cd.nodeIdx), x);

            lock.lock();
            m_DynamicLinearProjConstraints->push_back(std::move(pickProjector));
            lock.unlock();
    });
}
}
