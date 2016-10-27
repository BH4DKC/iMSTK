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

#include "imstkPbdModel.h"
#include "imstkTetrahedralMesh.h"
#include "imstkSurfaceMesh.h"
#include "imstkPbdVolumeConstraint.h"
#include "imstkPbdDistanceConstraint.h"
#include "imstkPbdDihedralConstraint.h"
#include "imstkPbdAreaConstraint.h"
#include "imstkPbdFETetConstraint.h"
#include "imstkPbdFEHexConstraint.h"

#include <g3log/g3log.hpp>

namespace imstk
{

bool
PositionBasedDynamicsModel::initializeFEMConstraints(FEMConstraint::MaterialType type)
{
    // check if constraint type matches the mesh type
    if (m_mesh->getType() != Geometry::Type::TetrahedralMesh)
    {
        LOG(WARNING) << "FEM Tetrahedral constraint should come with tetrahedral mesh";
        return false;
    }
    // ok, now create constraints
    auto tetMesh = std::static_pointer_cast<TetrahedralMesh>(m_mesh);
    std::vector<TetrahedralMesh::TetraArray> elements = tetMesh->getTetrahedraVertices();

    for (size_t k = 0; k < elements.size(); ++k)
    {
        TetrahedralMesh::TetraArray& tet = elements[k];

        auto c = std::make_shared<FEMTetConstraint>(type);
        c->initConstraint(*this, tet[0], tet[1], tet[2], tet[3]);
        m_constraints.push_back(c);
    }
    return true;
}

bool
PositionBasedDynamicsModel::initializeVolumeConstraints(const double& stiffness)
{
    // check if constraint type matches the mesh type
    if (m_mesh->getType() != Geometry::Type::TetrahedralMesh)
    {
        LOG(WARNING) << "Volume constraint should come with volumetric mesh";
        return false;
    }

    // ok, now create constraints
    auto tetMesh = std::static_pointer_cast<TetrahedralMesh>(m_mesh);
    std::vector<TetrahedralMesh::TetraArray> elements = tetMesh->getTetrahedraVertices();

    for (size_t k = 0; k < elements.size(); ++k)
    {
        TetrahedralMesh::TetraArray& tet = elements[k];

        auto c = std::make_shared<VolumeConstraint>();
        c->initConstraint(*this, tet[0], tet[1], tet[2], tet[3], stiffness);
        m_constraints.push_back(c);
    }
    return true;
}

bool
PositionBasedDynamicsModel::initializeDistanceConstraints(const double& stiffness)
{
    if (m_mesh->getType() == Geometry::Type::TetrahedralMesh)
    {
        auto tetMesh = std::static_pointer_cast<TetrahedralMesh>(m_mesh);
        int nV = tetMesh->getNumVertices();
        std::vector<std::vector<bool>> E(nV, std::vector<bool>(nV, 1));
        std::vector<TetrahedralMesh::TetraArray> elements = tetMesh->getTetrahedraVertices();

        for (size_t k = 0; k < elements.size(); ++k)
        {
            TetrahedralMesh::TetraArray& tet = elements[k];
            unsigned int i1;
            unsigned int i2;

            i1 = tet[0];
            i2 = tet[1];
            // check if added or not
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }

            i1 = tet[1];
            i2 = tet[2];
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }

            i1 = tet[2];
            i2 = tet[0];
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }

            i1 = tet[0];
            i2 = tet[3];
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }

            i1 = tet[1];
            i2 = tet[3];
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }

            i1 = tet[2];
            i2 = tet[3];
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }
        }
    }
    else if (m_mesh->getType() == Geometry::Type::SurfaceMesh)
    {
        auto triMesh = std::static_pointer_cast<SurfaceMesh>(m_mesh);
        int nV = triMesh->getNumVertices();
        std::vector<std::vector<bool>> E(nV, std::vector<bool>(nV, 1));
        std::vector<SurfaceMesh::TriangleArray> elements = triMesh->getTrianglesVertices();

        for (size_t k = 0; k < elements.size(); ++k)
        {
            SurfaceMesh::TriangleArray& tri = elements[k];
            unsigned int i1;
            unsigned int i2;

            i1 = tri[0];
            i2 = tri[1];
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }

            i1 = tri[1];
            i2 = tri[2];
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }

            i1 = tri[2];
            i2 = tri[0];
            if (E[i1][i2] && E[i2][i1])
            {
                auto c = std::make_shared<DistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(c);
                E[i1][i2] = 0;
            }
        }
    }
    return true;
}

bool
PositionBasedDynamicsModel::initializeAreaConstraints(const double& stiffness)
{
    // check if constraint type matches the mesh type
    if (m_mesh->getType() != Geometry::Type::SurfaceMesh)
    {
        LOG(WARNING) << "Area constraint should come with a triangular mesh";
        return false;
    }

    // ok, now create constraints
    auto triMesh = std::static_pointer_cast<SurfaceMesh>(m_mesh);
    std::vector<SurfaceMesh::TriangleArray> elements = triMesh->getTrianglesVertices();

    for (size_t k = 0; k < elements.size(); ++k)
    {
        SurfaceMesh::TriangleArray& tri = elements[k];

        auto c = std::make_shared<AreaConstraint>();
        c->initConstraint(*this, tri[0], tri[1], tri[2], stiffness);
        m_constraints.push_back(c);
    }
    return true;
}

bool
PositionBasedDynamicsModel::initializeDihedralConstraints(const double& stiffness)
{
    if (m_mesh->getType() != Geometry::Type::SurfaceMesh)
    {
        LOG(WARNING) << "Dihedral constraint should come with a triangular mesh";
        return false;
    }

    // ok, now create constraints
    auto triMesh = std::static_pointer_cast<SurfaceMesh>(m_mesh);
    std::vector<SurfaceMesh::TriangleArray> elements = triMesh->getTrianglesVertices();
    // following algorithm is terrible, should use half-edge instead
    std::vector<std::vector<unsigned int>> onering(triMesh->getNumVertices());

    for (size_t k = 0; k < elements.size(); ++k)
    {
        SurfaceMesh::TriangleArray& tri = elements[k];
        onering[tri[0]].push_back(k);
        onering[tri[1]].push_back(k);
        onering[tri[2]].push_back(k);
    }

    std::vector<std::vector<bool>> E(triMesh->getNumVertices(), std::vector<bool>(triMesh->getNumVertices(), 1));
    for (size_t k = 0; k < elements.size(); ++k)
    {
        SurfaceMesh::TriangleArray& tri = elements[k];
        std::vector<unsigned int>& r1 = onering[tri[0]];
        std::vector<unsigned int>& r2 = onering[tri[1]];
        std::vector<unsigned int>& r3 = onering[tri[2]];
        std::sort(r1.begin(), r1.end());
        std::sort(r2.begin(), r2.end());
        std::sort(r3.begin(), r3.end());
        std::vector<unsigned int> rs;
        std::vector<unsigned int>::iterator it;
        // check if processed or not
        if (E[tri[0]][tri[1]] && E[tri[1]][tri[0]])
        {
            rs.resize(2);
            it = std::set_intersection(r1.begin(), r1.end(), r2.begin(), r2.end(), rs.begin());
            rs.resize(it - rs.begin());
            if (rs.size() > 1)
            {
                int idx = (rs[0] == k)?1:0;
                SurfaceMesh::TriangleArray& t = elements[rs[idx]];
                for (int i = 0; i < 3; ++i)
                {
                    if (t[i] != tri[0] && t[i] != tri[1])
                    {
                        idx = i;
                        break;
                    }
                }
                auto c = std::make_shared<DihedralConstraint>();
                c->initConstraint(*this, tri[2], t[idx], tri[0], tri[1], stiffness);
                m_constraints.push_back(c);
            }
            E[tri[0]][tri[1]] = 0;
        }

        if (E[tri[1]][tri[2]] && E[tri[2]][tri[1]])
        {
            rs.resize(2);
            it = std::set_intersection(r2.begin(), r2.end(), r3.begin(), r3.end(), rs.begin());
            rs.resize(it - rs.begin());
            if (rs.size() > 1)
            {
                int idx = (rs[0] == k)?1:0;
                SurfaceMesh::TriangleArray& t = elements[rs[idx]];
                for (int i = 0; i < 3; ++i)
                {
                    if (t[i] != tri[1] && t[i] != tri[2])
                    {
                        idx = i;
                        break;
                    }
                }

                auto c = std::make_shared<DihedralConstraint>();
                c->initConstraint(*this, tri[0], t[idx], tri[1], tri[2], stiffness);
                m_constraints.push_back(c);
            }
            E[tri[1]][tri[2]] = 0;
        }

        if (E[tri[2]][tri[0]] && E[tri[0]][tri[2]])
        {
            rs.resize(2);
            it = std::set_intersection(r3.begin(), r3.end(), r1.begin(), r1.end(), rs.begin());
            rs.resize(it - rs.begin());
            if (rs.size() > 1)
            {
                int idx = (rs[0] == k)?1:0;
                SurfaceMesh::TriangleArray& t = elements[rs[idx]];
                for (int i = 0; i < 3; ++i)
                {
                    if (t[i] != tri[2] && t[i] != tri[0])
                    {
                        idx = i;
                        break;
                    }
                }

                auto c = std::make_shared<DihedralConstraint>();
                c->initConstraint(*this, tri[1], t[idx], tri[2], tri[0], stiffness);
                m_constraints.push_back(c);
            }
            E[tri[2]][tri[0]] = 0;
        }
    }
    return true;
}

void
PositionBasedDynamicsModel::projectConstraints()
{
    int i = 0;
    while (++i < m_maxIter)
    {
        for (auto c: m_constraints)
        {
            c->solvePositionConstraint(*this);
        }
    }
}

void
PositionBasedDynamicsModel::updatePhysicsGeometry()
{
    for (size_t i = 0; i < m_mesh->getNumVertices(); ++i)
    {
        m_mesh->setVerticePosition(i, m_state->getVertexPosition(i));
    }
}

void
PositionBasedDynamicsModel::updatePbdStateFromPhysicsGeometry()
{
    Vec3d pos;
    for (size_t i = 0; i < m_mesh->getNumVertices(); ++i)
    {
        pos = m_mesh->getVertexPosition(i);
        m_state->setVertexPosition(i, pos);
    }
}

void
PositionBasedDynamicsModel::setUniformMass(const double& val)
{
    if (val != 0.0)
    {
        std::fill(m_mass.begin(), m_mass.end(), val);
        std::fill(m_invMass.begin(), m_invMass.end(), 1 / val);
    }
    else
    {
        std::fill(m_invMass.begin(), m_invMass.end(), 0.0);
        std::fill(m_mass.begin(), m_mass.end(), 0.0);
    }
}

void
PositionBasedDynamicsModel::setParticleMass(const double& val, const unsigned int& idx)
{
    if (idx < m_mesh->getNumVertices())
    {
        m_mass[idx] = val;
        m_invMass[idx] = 1 / val;
    }
}

void
PositionBasedDynamicsModel::setFixedPoint(const unsigned int& idx)
{
    if (idx < m_mesh->getNumVertices())
    {
        m_invMass[idx] = 0;
    }
}

double
PositionBasedDynamicsModel::getInvMass(const unsigned int& idx)
{
    return m_invMass[idx];
}

void
PositionBasedDynamicsModel::integratePosition()
{
    auto& prevPos = m_state->getPreviousPositions();
    auto& pos = m_state->getPositions();
    auto& vel = m_state->getVelocities();
    auto& accn = m_state->getAccelerations();

    for (size_t i = 0; i < m_mesh->getNumVertices(); ++i)
    {
        if (m_invMass[i] != 0.0)
        {
            vel[i] += (accn[i] + m_gravity)*m_dt;
            prevPos[i] = pos[i];
            pos[i] += vel[i] * m_dt;
        }
    }
}

void
PositionBasedDynamicsModel::integrateVelocity()
{
    auto& prevPos = m_state->getPreviousPositions();
    auto& pos = m_state->getPositions();
    auto& vel = m_state->getVelocities();

    for (size_t i = 0; i < m_mesh->getNumVertices(); ++i)
    {
        if (m_invMass[i] != 0.0)
        {
            vel[i] = (pos[i] - prevPos[i]) / m_dt;
        }
    }
}

} // imstk
