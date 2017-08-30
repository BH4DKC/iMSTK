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

#include "imstkSurfaceMesh.h"

namespace imstk
{
void
SurfaceMesh::initialize(const StdVectorOfVec3d& vertices,
                        const std::vector<TriangleArray>& triangles,
                        const bool computeDerivedData)
{
    this->clear();

    PointSet::initialize(vertices);
    this->setTrianglesVertices(triangles);

    if (computeDerivedData)
    {
        this->computeVertexNormals();
    }
}

void
SurfaceMesh::initialize(const StdVectorOfVec3d& vertices,
                        const std::vector<TriangleArray>& triangles,
                        const StdVectorOfVec3d& normals,
                        const bool computeDerivedData)
{
    this->initialize(vertices, triangles, computeDerivedData);

    this->m_vertexNormals = normals;
    this->computeUVSeamVertexGroups();

    this->computeVertexNormals();
}

void
SurfaceMesh::clear()
{
    PointSet::clear();
    m_trianglesVertices.clear();
    m_vertexNeighborTriangles.clear();
    m_vertexNeighborVertices.clear();
    m_triangleNormals.clear();
    m_vertexNormals.clear();
}

void
SurfaceMesh::print() const
{
    PointSet::print();

    LOG(INFO) << "Number of triangles: " << this->getNumTriangles();
    LOG(INFO) << "Triangles:";
    for (auto &tri : m_trianglesVertices)
    {
        LOG(INFO) << tri.at(0) << ", " << tri.at(1) << ", " << tri.at(2);
    }
}

double
SurfaceMesh::getVolume() const
{
    // TODO
    // 1. Check for water tightness
    // 2. Compute volume based on signed distance

    LOG(WARNING) << "SurfaceMesh::getVolume error: not implemented.";
    return 0.0;
}

void
SurfaceMesh::computeVertexNeighborTriangles()
{
    m_vertexNeighborTriangles.resize(m_vertexPositions.size());

    size_t triangleId = 0;

    for (const auto& t : m_trianglesVertices)
    {
        m_vertexNeighborTriangles.at(t.at(0)).insert(triangleId);
        m_vertexNeighborTriangles.at(t.at(1)).insert(triangleId);
        m_vertexNeighborTriangles.at(t.at(2)).insert(triangleId);
        triangleId++;
    }
}

void
SurfaceMesh::computeVertexNeighborVertices()
{
    m_vertexNeighborVertices.resize(m_vertexPositions.size());

    if (m_vertexNeighborTriangles.empty())
    {
        this->computeVertexNeighborTriangles();
    }

    for (size_t vertexId = 0; vertexId < m_vertexNeighborVertices.size(); ++vertexId)
    {
        for (const size_t& triangleId : m_vertexNeighborTriangles.at(vertexId))
        {
            for (const size_t& vertexId2 : m_trianglesVertices.at(triangleId))
            {
                if (vertexId2 != vertexId)
                {
                    m_vertexNeighborVertices.at(vertexId).insert(vertexId2);
                }
            }
        }
    }
}

void
SurfaceMesh::computeTrianglesNormals()
{
    m_triangleNormals.resize(m_trianglesVertices.size());

    for (size_t triangleId = 0; triangleId < m_triangleNormals.size(); ++triangleId)
    {
        const auto& t  = m_trianglesVertices.at(triangleId);
        const auto& p0 = m_vertexPositions.at(t.at(0));
        const auto& p1 = m_vertexPositions.at(t.at(1));
        const auto& p2 = m_vertexPositions.at(t.at(2));

        m_triangleNormals.at(triangleId) = ((p1 - p0).cross(p2 - p0)).normalized();
    }
}

void
SurfaceMesh::computeVertexNormals()
{
    m_vertexNormals.resize(m_vertexPositions.size());

    if (m_vertexNeighborTriangles.empty())
    {
        this->computeVertexNeighborTriangles();
    }

    this->computeTrianglesNormals();

    StdVectorOfVec3d temp_normals(m_vertexNormals.size());
    for (size_t vertexId = 0; vertexId < m_vertexNormals.size(); ++vertexId)
    {
        temp_normals[vertexId] = Vec3d(0, 0, 0);
        for (const size_t& triangleId : m_vertexNeighborTriangles.at(vertexId))
        {
            temp_normals[vertexId] += m_triangleNormals[triangleId];
        }
    }

    // Correct for UV seams
    for (size_t vertexId = 0; vertexId < m_vertexNormals.size(); ++vertexId)
    {
        NormalGroup group = {m_vertexPositions[vertexId], m_vertexNormals[vertexId]};

        m_vertexNormals[vertexId] = temp_normals[vertexId];

        if (m_UVSeamVertexGroups.find(group) == m_UVSeamVertexGroups.end())
        {
            m_vertexNormals[vertexId].normalize();
            continue;
        }

        auto seamGroup = *m_UVSeamVertexGroups[group].get();

        for (auto index : seamGroup)
        {
            m_vertexNormals[vertexId] += temp_normals[index];
        }

        m_vertexNormals[vertexId].normalize();
    }
}

void
SurfaceMesh::optimizeForDataLocality()
{
    const size_t numVertices = this->getNumVertices();
    const size_t numTriangles = this->getNumTriangles();

    // First find the list of triangles a given vertex is part of
    std::vector<std::vector<size_t>> vertexNeighbors;
    vertexNeighbors.resize(this->getNumVertices());
    size_t triId = 0;
    for (const auto &tri : this->getTrianglesVertices())
    {
        vertexNeighbors[tri[0]].push_back(triId);
        vertexNeighbors[tri[1]].push_back(triId);
        vertexNeighbors[tri[2]].push_back(triId);

        triId++;
    }

    std::vector<TriangleArray> optimizedConnectivity;
    std::vector<size_t> optimallyOrderedNodes;
    std::list<size_t> triUnderConsideration;
    std::vector<bool> isNodeAdded(numVertices, false);
    std::vector<bool> isTriangleAdded(numTriangles, false);
    std::list<size_t> newlyAddedNodes;

    // A. Initialize
    optimallyOrderedNodes.push_back(0);
    isNodeAdded.at(0) = true;
    for (const auto &neighTriId : vertexNeighbors[0])
    {
        triUnderConsideration.push_back(neighTriId);
    }

    // B. Iterate till all the nodes are added to optimized mesh
    size_t vertId[3];
    auto connectivity = this->getTrianglesVertices();
    while (triUnderConsideration.size() != 0)
    {
        // B.1 Add new nodes and triangles
        for (const auto &triId : triUnderConsideration)
        {
            for (int i = 0; i < 3; ++i)
            {
                if (!isNodeAdded.at(connectivity.at(triId)[i]))
                {
                    optimallyOrderedNodes.push_back(connectivity.at(triId)[i]);
                    isNodeAdded.at(connectivity.at(triId)[i]) = true;
                    newlyAddedNodes.push_back(connectivity.at(triId)[i]);
                }
                vertId[i] = *std::find(optimallyOrderedNodes.begin(),
                                       optimallyOrderedNodes.end(),
                                       connectivity.at(triId)[i]);
            }
            TriangleArray tmpTri = { { vertId[0], vertId[1], vertId[2] } };
            optimizedConnectivity.push_back(tmpTri);
            isTriangleAdded.at(triId) = true;
        }

        // B.2 Setup triangles to be considered for next iteration
        triUnderConsideration.clear();
        for (const auto &newNodes : newlyAddedNodes)
        {
            for (const auto &neighTriId : vertexNeighbors[newNodes])
            {
                if (!isTriangleAdded[neighTriId])
                {
                    triUnderConsideration.push_back(neighTriId);
                }
            }
        }
        triUnderConsideration.sort();
        triUnderConsideration.unique();

        newlyAddedNodes.clear();
    }

    // C. Initialize this mesh with the newly computed ones
    StdVectorOfVec3d optimallyOrderedNodalPos;
    std::vector<TriangleArray> optConnectivityRenumbered;

    // C.1 Get the positions
    for (const auto &nodalId : optimallyOrderedNodes)
    {
        optimallyOrderedNodalPos.push_back(this->getInitialVertexPosition(nodalId));
    }

    // C.2 Get the renumbered connectivity
    for (size_t i = 0; i < numTriangles; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            vertId[j] = (std::find(optimallyOrderedNodes.begin(),
                                   optimallyOrderedNodes.end(),
                                   optimizedConnectivity.at(i)[j]) -
                         optimallyOrderedNodes.begin());
        }

        TriangleArray tmpTriArray = { { vertId[0], vertId[1], vertId[2] } };
        optConnectivityRenumbered.push_back(tmpTriArray);
    }

    // D. Assign the rewired mesh data to the mesh
    this->initialize(optimallyOrderedNodalPos, optConnectivityRenumbered);
}

const std::vector<SurfaceMesh::TriangleArray>&
SurfaceMesh::getTrianglesVertices() const
{
    return m_trianglesVertices;
}

void
SurfaceMesh::setTrianglesVertices(const std::vector<TriangleArray>& triangles)
{
    m_trianglesVertices = triangles;
}

const StdVectorOfVec3d&
SurfaceMesh::getTriangleNormals() const
{
    return m_triangleNormals;
}

const Vec3d&
SurfaceMesh::getTriangleNormal(size_t i) const
{
    return m_triangleNormals.at(i);
}

void
SurfaceMesh::setVertexNormals(const StdVectorOfVec3d& normals)
{
    m_vertexNormals = normals;
}

const StdVectorOfVec3d&
SurfaceMesh::getVertexNormals() const
{
    return m_vertexNormals;
}

StdVectorOfVec3d&
SurfaceMesh::getVertexNormalsNotConst()
{
    return m_vertexNormals;
}

void
SurfaceMesh::setVertexTangents(const StdVectorOfVec3d& tangents)
{
    m_vertexTangents = tangents;
}

const StdVectorOfVec3d&
SurfaceMesh::getVertexTangents() const
{
    return m_vertexTangents;
}

void
SurfaceMesh::setVertexBitangents(const StdVectorOfVec3d& bitangents)
{
    m_vertexBitangents = bitangents;
}

const StdVectorOfVec3d&
SurfaceMesh::getVertexBitangents() const
{
    return m_vertexBitangents;
}

size_t
SurfaceMesh::getNumTriangles() const
{
    return this->m_trianglesVertices.size();
}

void
SurfaceMesh::setDefaultTCoords(std::string arrayName)
{
    m_defaultTCoords = arrayName;
}

std::string
SurfaceMesh::getDefaultTCoords()
{
    return m_defaultTCoords;
}

void
SurfaceMesh::flipNormals()
{
    for (auto& tri : m_trianglesVertices)
    {
        auto temp = tri[0];
        tri[0] = tri[1];
        tri[1] = temp;
    }
}

void
SurfaceMesh::correctWindingOrder()
{
    // Enforce consistency in winding of a particular triangle with its neighbor (master)
    auto enforceWindingConsistency =
        [this](const size_t masterTriId, const size_t neighTriId)
        {
            const auto& masterTri = m_trianglesVertices[masterTriId];
            auto& neighTri = m_trianglesVertices[neighTriId];

            for (unsigned int l = 0; l < 3; ++l)
            {
                for (unsigned int k = 0; k < 3; ++k)
                {
                    if (masterTri[k] == neighTri[l] && masterTri[(k + 1) % 3] == neighTri[(l + 1) % 3])
                    {
                        // Flip the order of neighbor triangle
                        auto tempId = neighTri[0];
                        neighTri[0] = neighTri[1];
                        neighTri[1] = tempId;
                        break;
                    }
                }
            }
        };

    // Search for triangle neighbors that share a common edge
    auto getTriangleNeighbors =
        [this](const size_t triID, int* neig)
        {
            const auto& currentTri = m_trianglesVertices[triID];
            size_t currentId = 0;
            int numNeigh = 0;
            for (auto& tri : m_trianglesVertices)
            {
                if (triID == currentId)
                {
                    currentId++;
                    continue;
                }

                int numCommon = 0;
                for (int i = 0; i < 3; ++i)
                {
                    if (currentTri[i] == tri[0] || currentTri[i] == tri[1] || currentTri[i] == tri[2])
                    {
                        numCommon++;
                        if (numCommon == 2)
                        {
                            neig[numNeigh] = (int)currentId;
                            numNeigh++;

                            if (numNeigh == 3)
                            {
                                return;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                currentId++;
            }
        };

    // Start with a reference triangle and enforce the consistency of its neighbors
    // Keep track of those neighbor triangles whose order is enforced but its neighbors not
    // necessarily enforced (trianglesCorrected). Continue this until there is no
    // triangle left in the list
    std::vector<bool> trianglesCorrected(this->getNumTriangles(), false);
    std::vector<size_t> correctedTriangles;

    size_t currentTriangle = 0; // Start with triangle 0
    correctedTriangles.push_back(currentTriangle);
    trianglesCorrected[currentTriangle] = true;
    do
    {
        currentTriangle = correctedTriangles[0];
        int neighborTri[3] = {-1, -1, -1};
        getTriangleNeighbors(currentTriangle, &neighborTri[0]);

        for (int i = 0; i < 3; ++i)
        {
            if (neighborTri[i] >= 0 && !trianglesCorrected[neighborTri[i]])
            {
                enforceWindingConsistency(currentTriangle, neighborTri[i]);

                correctedTriangles.push_back(neighborTri[i]);
                trianglesCorrected[neighborTri[i]] = true;
            }
        }

        correctedTriangles.erase(std::remove(correctedTriangles.begin(),
                                             correctedTriangles.end(),
                                             currentTriangle),
                                 correctedTriangles.end());
    }
    while (correctedTriangles.size() > 0);
}

void
SurfaceMesh::computeUVSeamVertexGroups()
{
    // Reset vertex groups
    m_UVSeamVertexGroups.clear();

    // Initial pass to bin vertices based on positions
    for (size_t i = 0; i < m_vertexPositions.size(); i++)
    {
        NormalGroup group = {m_vertexPositions[i], m_vertexNormals[i]};

        if (m_UVSeamVertexGroups.find(group) == m_UVSeamVertexGroups.end())
        {
            m_UVSeamVertexGroups.insert(std::pair<NormalGroup, std::shared_ptr<std::vector<size_t>>>(
                group, std::make_shared<std::vector<size_t>>()));
        }
        m_UVSeamVertexGroups[group]->push_back(i);
    }
}
} // imstk