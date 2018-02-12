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

#include "imstkMultiMeshBoneSawingCH.h"

#include "imstkCollidingObject.h"
#include "imstkTetrahedralMesh.h"
#include "imstkCollisionData.h"
#include "imstkDeviceTracker.h"
#include "imstkMath.h"

#include <g3log/g3log.hpp>

// define this if a conservative element removal strategy is needed
#define TETRA_REMOVAL_STRATEGY_CONSERVATIVE_

namespace imstk
{
MultiMeshBoneSawingCH::MultiMeshBoneSawingCH(const Side& side,
                           const CollisionData& colData,
                           std::vector<std::shared_ptr<CollidingObject>>& bone,
                           std::shared_ptr<CollidingObject> saw) :
    CollisionHandling(Type::BoneDrilling, side, colData),
    m_saw(saw),
    m_bone(bone)
{
    const auto numPar = m_bone.size();
    m_nodalDensity.resize(numPar);
    m_nodeRemovalStatus.resize(numPar);
    m_nodalCardinalSet.resize(numPar);
    m_erodedNodes.resize(numPar);

    for (size_t meshPar = 0; meshPar < numPar; ++meshPar)
    {
        m_erodedNodes[meshPar].resize(0);

        auto boneMesh = std::dynamic_pointer_cast<TetrahedralMesh>(m_bone[meshPar]->getCollidingGeometry());

        if (!boneMesh)
        {
            LOG(WARNING) << "BoneDrillingCH::BoneDrillingCH Error:The bone colliding geometry is not a mesh!";
        }

        // Initialize bone density values        
        for (int i = 0; i < boneMesh->getNumVertices(); ++i)
        {
            m_nodalDensity[meshPar].push_back(m_initialBoneDensity);
        }

        for (int i = 0; i < boneMesh->getNumVertices(); ++i)
        {
            m_nodeRemovalStatus[meshPar].push_back(false);
        }

        m_nodalCardinalSet[meshPar].resize(boneMesh->getNumVertices());
        for (int i = 0; i < boneMesh->getNumVertices(); ++i)
        {
            std::vector<int> row;
            m_nodalCardinalSet[meshPar].push_back(row);
        }

        // Pre-compute the nodal cardinality set
        for (int i = 0; i < boneMesh->getNumTetrahedra(); ++i)
        {
            for (auto& vert : boneMesh->getTetrahedronVertices(i))
            {
                m_nodalCardinalSet[meshPar][vert].push_back(i);
            }
        }
    }    
}

void
MultiMeshBoneSawingCH::erodeBone()
{
    for (auto n = 0; n < m_colData.MGAColData.size(); ++n)
    {
        auto cd = m_colData.MGAColData[n];
        if (m_nodeRemovalStatus[cd.meshId][cd.nodeId])
        {
            continue;
        }

        m_nodalDensity[cd.meshId][cd.nodeId] -= 0.001*(m_angularSpeed / m_BoneHardness)*m_stiffness*cd.penetrationVector.norm()*0.001 * 1000;

#ifdef TETRA_REMOVAL_STRATEGY_CONSERVATIVE
        if (m_nodalDensity[cd.meshId][cd.nodeId] <= 0.2)
        {
            m_erodedNodes[cd.meshId].push_back(cd.nodeId);
            m_nodeRemovalStatus[cd.meshId][cd.nodeId] = true;

            // tag the tetra that will be removed
            for (auto& tetId : m_nodalCardinalSet[cd.meshId][cd.nodeId])
            {
                auto boneTetMesh = std::dynamic_pointer_cast<TetrahedralMesh>(m_bone[cd.meshId]->getCollidingGeometry());

                auto ss = boneTetMesh->getTetrahedronVertices(tetId);

                int count = 0;
                for (int i = 0; i < 4; i++)
                {
                    if (m_nodeRemovalStatus[cd.meshId][ss[i]])
                    {
                        count++;
                    }
                }
                if (count >= 2)
                {
                    boneTetMesh->setTetrahedraAsRemoved(tetId);
                    boneTetMesh->setTopologyChangedFlag(true);
                }
            }
        }
#else
        bool topologyChanged = false;
        if (m_nodalDensity[cd.meshId][cd.nodeId] <= 0.)
        {
            m_erodedNodes[cd.meshId].push_back(cd.nodeId);
            m_nodeRemovalStatus[cd.meshId][cd.nodeId] = true;
            
            if (auto boneTetMesh = std::dynamic_pointer_cast<TetrahedralMesh>(m_bone[cd.meshId]->getCollidingGeometry()))
            {
                // tag the tetra that will be removed
                for (auto& tetId : m_nodalCardinalSet[cd.meshId][cd.nodeId])
                {
                    boneTetMesh->setTetrahedraAsRemoved(tetId);
                    topologyChanged = true;
                }
                if (topologyChanged)
                {
                    boneTetMesh->setTopologyChangedFlag(true);
                }
            }
        }
#endif
    }
}

void
MultiMeshBoneSawingCH::computeContactForces()
{
    // Check if any collisions
    const auto collGeoPosition = m_saw->getCollidingGeometry()->getTranslation();

    m_saw->getVisualGeometry()->setRotation(m_saw->getCollidingGeometry()->getRotation());

    if (m_colData.MGAColData.empty())
    {
        // Set the visual object position same as the colliding object position
        m_saw->getVisualGeometry()->setTranslation(collGeoPosition);
        return;
    }
    // Update visual object position

    // Aggregate collision data
    Vec3d t = Vec3d::Zero();
    double maxDepth = -MAX_D;

    for (auto n = 0; n < m_colData.MGAColData.size(); ++n)
    {
        auto cd = m_colData.MGAColData[n];
        if (m_nodeRemovalStatus[cd.meshId][cd.nodeId])
        {
            continue;
        }

        if (cd.penetrationVector.norm() > maxDepth)
        {
            maxDepth = cd.penetrationVector.norm();
            t = cd.penetrationVector;
        }
    }
    //std::cout << "Max. " << maxDepth << std::endl;
    m_saw->getVisualGeometry()->setTranslation(collGeoPosition + t);

    // Spring force
    Vec3d force = m_stiffness * t;

    // Damping force
    const double dt = 0.1; // Time step size to calculate the object velocity
    force += m_initialStep ? Vec3d(0.0, 0.0, 0.0) : (-m_damping / dt)*(t - m_prevPos);

    // Update object contact force
    m_saw->appendForce(force);

    // Decrease the density at the nodal points and remove if the density goes below 0
    this->erodeBone();
    

    // Housekeeping
    m_initialStep = false;
    m_prevPos = t;
}
}// iMSTK