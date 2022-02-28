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

#include "imstkOneToOneMap.h"
#include "imstkParallelUtils.h"
#include "imstkLogger.h"
#include "imstkPointSet.h"

namespace imstk
{
OneToOneMap::OneToOneMap(std::shared_ptr<Geometry> parent, std::shared_ptr<Geometry> child)
{
    setParentGeometry(parent);
    setChildGeometry(child);
}

void
OneToOneMap::compute()
{
    CHECK(m_parentGeom != nullptr && m_childGeom != nullptr) << "OneToOneMap map is being applied without valid geometries";

    auto meshParent = std::dynamic_pointer_cast<PointSet>(m_parentGeom);
    auto meshChild  = std::dynamic_pointer_cast<PointSet>(m_childGeom);

    m_oneToOneMap.clear();
    ParallelUtils::SpinLock lock;

    std::shared_ptr<VecDataArray<double, 3>> parentVerticesPtr = meshParent->getInitialVertexPositions();
    const VecDataArray<double, 3>&           parentVertices    = *parentVerticesPtr;
    std::shared_ptr<VecDataArray<double, 3>> childVerticesPtr  = meshChild->getInitialVertexPositions();
    const VecDataArray<double, 3>&           childVertices     = *childVerticesPtr;

    // For every vertex on the child, find corresponding one on the parent
    ParallelUtils::parallelFor(meshChild->getNumVertices(),
        [&](const int nodeId)
        {
            // Find the enclosing or closest tetrahedron
            IndexType matchingNodeId = findMatchingVertex(parentVertices, childVertices[nodeId]);
            if (matchingNodeId == IMSTK_NO_INDEX)
            {
                return;
            }

            // Add to the map
            // Note: This replaces the map if one with <nodeId> already exists
            lock.lock();
            m_oneToOneMap[nodeId] = matchingNodeId; // child index -> parent index
            lock.unlock();
        });

    // Copy data from map to vector for parallel processing
    m_oneToOneMapVector.clear();
    for (auto kv : m_oneToOneMap)
    {
        m_oneToOneMapVector.push_back({ kv.first, kv.second });
    }
    std::sort(m_oneToOneMapVector.begin(), m_oneToOneMapVector.end());
}

IndexType
OneToOneMap::findMatchingVertex(const VecDataArray<double, 3>& parentVertices, const Vec3d& p)
{
    for (int idx = 0; idx < parentVertices.size(); ++idx)
    {
        if (p.isApprox(parentVertices[idx], m_epsilon)) return idx;
    }
    return IMSTK_NO_INDEX;
}

bool
OneToOneMap::isValid() const
{
    auto meshParent = std::dynamic_pointer_cast<PointSet>(m_parentGeom);
    auto meshChild  = std::dynamic_pointer_cast<PointSet>(m_childGeom);

    return meshParent != nullptr && meshChild != nullptr;
}

void
OneToOneMap::setMap(const std::unordered_map<IndexType, IndexType>& sourceMap)
{
    m_oneToOneMap = sourceMap;

    // Copy data from map to vector for parallel processing
    m_oneToOneMapVector.clear();
    for (auto kv : m_oneToOneMap)
    {
        m_oneToOneMapVector.push_back({ kv.first, kv.second });
    }
    std::sort(m_oneToOneMapVector.begin(), m_oneToOneMapVector.end());
}

void
OneToOneMap::apply()
{
    // Check Map active
    if (!m_isActive)
    {
        LOG(WARNING) << "OneToOneMap map is not active";
        return;
    }

    // Check geometries
    CHECK(m_parentGeom != nullptr && m_childGeom != nullptr) << "OneToOneMap map is being applied without valid geometries";

    // Check data
    CHECK(m_oneToOneMap.size() == m_oneToOneMapVector.size()) << "Internal data is corrupted";

    auto meshParent = std::dynamic_pointer_cast<PointSet>(m_parentGeom);
    auto meshChild  = std::dynamic_pointer_cast<PointSet>(m_childGeom);

    CHECK(meshParent != nullptr && meshChild != nullptr) << "Failed to cast from Geometry to PointSet";

    VecDataArray<double, 3>&       childVertices  = *meshChild->getVertexPositions();
    const VecDataArray<double, 3>& parentVertices = *meshParent->getVertexPositions();
    ParallelUtils::parallelFor(m_oneToOneMapVector.size(),
        [&](const size_t idx) {
            const auto& mapValue = m_oneToOneMapVector[idx];
            childVertices[mapValue.first] = parentVertices[mapValue.second];
        });
    meshChild->postModified();
}

void
OneToOneMap::print() const
{
    // Print Type
    GeometryMap::print();

    // Print the one-to-one map
    LOG(INFO) << "[childVertId, parentVertexId]\n";
    for (auto const& mapValue : m_oneToOneMap)
    {
        LOG(INFO) << "[" << mapValue.first << ", " << mapValue.second << "]\n";
    }
}

void
OneToOneMap::setParentGeometry(std::shared_ptr<Geometry> parent)
{
    CHECK(parent != nullptr) << "Can't set parent to nullptr";
    auto pointSet = std::dynamic_pointer_cast<PointSet>(parent);

    CHECK(pointSet != nullptr) << "The geometry provided is not a PointSet!";

    if (pointSet == getChildGeometry())
    {
        LOG(WARNING) << "Parent and Child geometry are the same.";
    }

    GeometryMap::setParentGeometry(parent);
}

void
OneToOneMap::setChildGeometry(std::shared_ptr<Geometry> child)
{
    CHECK(child != nullptr) << "Can't set child to nullptr";
    auto pointSet = std::dynamic_pointer_cast<PointSet>(child);
    CHECK(pointSet != nullptr) << "The geometry provided is not a PointSet!\n";
    if (pointSet == getParentGeometry())
    {
        LOG(WARNING) << "Parent and Child geometry are the same.";
    }
    GeometryMap::setChildGeometry(child);
}

IndexType
OneToOneMap::getMapIdx(const IndexType idx) const
{
    auto found = m_oneToOneMap.find(idx);
    return (found != m_oneToOneMap.cend()) ? found->second : IMSTK_NO_INDEX;
}
} // namespace imstk