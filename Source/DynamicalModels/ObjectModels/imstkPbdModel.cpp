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

#include "imstkGraph.h"
#include "imstkPbdModel.h"
#include "imstkTetrahedralMesh.h"
#include "imstkSurfaceMesh.h"
#include "imstkSurfaceCuttingManager.h"
#include "imstkPbdVolumeConstraint.h"
#include "imstkPbdDistanceConstraint.h"
#include "imstkPbdDihedralConstraint.h"
#include "imstkPbdAreaConstraint.h"
#include "imstkPbdFETetConstraint.h"
#include "imstkPbdFEHexConstraint.h"
#include "imstkPbdConstantDensityConstraint.h"
#include "imstkParallelUtils.h"

#include <g3log/g3log.hpp>

#include <unordered_map>

namespace imstk
{
void
PBDModelConfig::enableConstraint(PbdConstraint::Type type, double stiffness)
{
    LOG_IF(FATAL, (type == PbdConstraint::Type::FEMTet || type == PbdConstraint::Type::FEMHex))
        << "FEM constraint should be enabled by the enableFEMConstraint function";
    m_RegularConstraints.push_back({ type, stiffness });
}

void
PBDModelConfig::enableFEMConstraint(PbdConstraint::Type type, PbdFEMConstraint::MaterialType material)
{
    LOG_IF(FATAL, (type != PbdConstraint::Type::FEMTet && type != PbdConstraint::Type::FEMHex))
        << "Non-FEM constraint should be enabled by the enableConstraint function";
    m_FEMConstraints.push_back({ type, material });
}

void
PbdModel::configure(const std::shared_ptr<PBDModelConfig>& params)
{
    LOG_IF(FATAL, (!this->getModelGeometry())) << "PbdModel::configure - Set PBD Model geometry before configuration!";

    m_Parameters = params;
    this->setNumDegreeOfFreedom(this->getModelGeometry()->getNumVertices() * 3);
}

bool
PbdModel::initialize()
{
    LOG_IF(FATAL, (!this->getModelGeometry())) << "Model geometry is not yet set! Cannot initialize without model geometry.";

    m_initialState  = std::make_shared<PbdState>();
    m_previousState = std::make_shared<PbdState>();
    m_currentState  = std::make_shared<PbdState>();

    bool option[3] = { 1, 0, 0 };
    m_initialState->initialize(m_mesh, option);
    m_previousState->initialize(m_mesh, option);

    option[1] = option[2] = 1;
    m_currentState->initialize(m_mesh, option);

    m_initialState->setPositions(m_mesh->getVertexPositions());
    m_currentState->setPositions(m_mesh->getVertexPositions());

    auto numParticles = m_mesh->getNumVertices();
    m_mass.resize(numParticles, 0);
    m_invMass.resize(numParticles, 0);
    setUniformMass(m_Parameters->m_uniformMassValue);

    for (auto i : m_Parameters->m_fixedNodeIds)
    {
        setFixedPoint(i);
    }
    //Increase viscousDampingCoeff to resolve the oscillation issue in cutting.
    m_Parameters->m_viscousDampingCoeff = 0.1;
    bool bOK = true; // Return immediately if some constraint failed to initialize

    // Initialize FEM constraints
    for (auto& constraint: m_Parameters->m_FEMConstraints)
    {
        computeElasticConstants();
        if (!initializeFEMConstraints(constraint.second))
        {
            return false;
        }
    }

    // Initialize other constraints
    for (auto& constraint: m_Parameters->m_RegularConstraints)
    {
        if (!bOK)
        {
            return false;
        }
        switch (constraint.first)
        {
        case PbdConstraint::Type::Volume:
            bOK = initializeVolumeConstraints(constraint.second);
            break;

        case PbdConstraint::Type::Distance:
            bOK = initializeDistanceConstraints(constraint.second);
            m_stiffnessStreching = constraint.second;
            break;

        case PbdConstraint::Type::Area:
            bOK = initializeAreaConstraints(constraint.second);
            break;

        case PbdConstraint::Type::Dihedral:
            bOK = initializeDihedralConstraints(constraint.second);
            m_stiffnessBending = constraint.second;
            break;

        case PbdConstraint::Type::ConstantDensity:
            bOK = initializeConstantDensityConstraint(constraint.second);
            break;

        default:
            LOG(FATAL) << "Invalid constraint type";
        }
    }

    // Partition constraints for parallel computation
     partitionCostraints();

    return bOK;
}

void
PbdModel::computeElasticConstants()
{
    if (std::abs(m_Parameters->m_mu) < MIN_REAL
        && std::abs(m_Parameters->m_lambda) < MIN_REAL)
    {
        const auto E  = m_Parameters->m_YoungModulus;
        const auto nu = m_Parameters->m_PoissonRatio;
        m_Parameters->m_mu     = E / Real(2.0) / (Real(1.0) + nu);
        m_Parameters->m_lambda = E * nu / ((Real(1.0) + nu) * (Real(1.0) - Real(2.0) * nu));
    }
    else
    {
        const auto mu     = m_Parameters->m_mu;
        const auto lambda = m_Parameters->m_lambda;
        m_Parameters->m_YoungModulus = mu * (Real(3.0) * lambda + Real(2.0) * mu) / (lambda + mu);
        m_Parameters->m_PoissonRatio = lambda / Real(2.0) / (lambda + mu);
    }
}

bool
PbdModel::initializeFEMConstraints(PbdFEMConstraint::MaterialType type)
{
    // Check if constraint type matches the mesh type
    if (m_mesh->getType() != Geometry::Type::TetrahedralMesh)
    {
        LOG(WARNING) << "FEM Tetrahedral constraint should come with tetrahedral mesh";
        return false;
    }

    // Create constraints
    const auto& tetMesh  = std::static_pointer_cast<TetrahedralMesh>(m_mesh);
    const auto& elements = tetMesh->getTetrahedraVertices();

    ParallelUtils::SpinLock lock;
    ParallelUtils::parallelFor(elements.size(),
        [&](const size_t k)
        {
            auto& tet = elements[k];
            auto c = std::make_shared<PbdFEMTetConstraint>(type);
            c->initConstraint(*this, tet[0], tet[1], tet[2], tet[3]);
            lock.lock();
            m_constraints.push_back(std::move(c));
            lock.unlock();
        });
    return true;
}

bool
PbdModel::initializeVolumeConstraints(const double stiffness)
{
    // Check if constraint type matches the mesh type
    if (m_mesh->getType() != Geometry::Type::TetrahedralMesh)
    {
        LOG(WARNING) << "Volume constraint should come with volumetric mesh";
        return false;
    }

    // Create constraints
    const auto& tetMesh  = std::static_pointer_cast<TetrahedralMesh>(m_mesh);
    const auto& elements = tetMesh->getTetrahedraVertices();

    ParallelUtils::SpinLock lock;
    ParallelUtils::parallelFor(elements.size(),
        [&](const size_t k)
        {
            auto& tet = elements[k];
            auto c = std::make_shared<PbdVolumeConstraint>();
            c->initConstraint(*this, tet[0], tet[1], tet[2], tet[3], stiffness);
            lock.lock();
            m_constraints.push_back(std::move(c));
            lock.unlock();
        });
    return true;
}

bool
PbdModel::initializeDistanceConstraints(const double stiffness)
{
    auto addConstraint =
        [&](std::vector<std::vector<bool>>& E, size_t i1, size_t i2)
        {
            if (i1 > i2)     // Make sure i1 is always smaller than i2
            {
                std::swap(i1, i2);
            }
            if (E[i1][i2])
            {
                E[i1][i2] = 0;
                auto c = std::make_shared<PbdDistanceConstraint>();
                c->initConstraint(*this, i1, i2, stiffness);
                m_constraints.push_back(std::move(c));
            }
        };

    if (m_mesh->getType() == Geometry::Type::TetrahedralMesh)
    {
        const auto&                    tetMesh  = std::static_pointer_cast<TetrahedralMesh>(m_mesh);
        const auto&                    elements = tetMesh->getTetrahedraVertices();
        const auto                     nV       = tetMesh->getNumVertices();
        std::vector<std::vector<bool>> E(nV, std::vector<bool>(nV, 1));

        for (size_t k = 0; k < elements.size(); ++k)
        {
            auto& tet = elements[k];
            addConstraint(E, tet[0], tet[1]);
            addConstraint(E, tet[0], tet[2]);
            addConstraint(E, tet[0], tet[3]);
            addConstraint(E, tet[1], tet[2]);
            addConstraint(E, tet[1], tet[3]);
            addConstraint(E, tet[2], tet[3]);
        }
    }
    else if (m_mesh->getType() == Geometry::Type::SurfaceMesh)
    {
        const auto&                    triMesh  = std::static_pointer_cast<SurfaceMesh>(m_mesh);
        const auto&                    elements = triMesh->getTrianglesVertices();
        const auto                     nV       = triMesh->getNumVertices();
        std::vector<std::vector<bool>> E(nV, std::vector<bool>(nV, 1));

        for (size_t k = 0; k < elements.size(); ++k)
        {
            auto& tri = elements[k];
            addConstraint(E, tri[0], tri[1]);
            addConstraint(E, tri[0], tri[2]);
            addConstraint(E, tri[1], tri[2]);
        }
    }

    return true;
}

bool
PbdModel::initializeAreaConstraints(const double stiffness)
{
    // check if constraint type matches the mesh type
    if (m_mesh->getType() != Geometry::Type::SurfaceMesh)
    {
        LOG(WARNING) << "Area constraint should come with a triangular mesh";
        return false;
    }

    // ok, now create constraints
    const auto& triMesh  = std::static_pointer_cast<SurfaceMesh>(m_mesh);
    const auto& elements = triMesh->getTrianglesVertices();

    ParallelUtils::SpinLock lock;
    ParallelUtils::parallelFor(elements.size(),
        [&](const size_t k)
        {
            auto& tri = elements[k];
            auto c = std::make_shared<PbdAreaConstraint>();
            c->initConstraint(*this, tri[0], tri[1], tri[2], stiffness);
            lock.lock();
            m_constraints.push_back(std::move(c));
            lock.unlock();
        });
    return true;
}

bool
PbdModel::initializeDihedralConstraints(const double stiffness)
{
    if (m_mesh->getType() != Geometry::Type::SurfaceMesh)
    {
        LOG(WARNING) << "Dihedral constraint should come with a triangular mesh";
        return false;
    }

    // Create constraints
    const auto&                      triMesh  = std::static_pointer_cast<SurfaceMesh>(m_mesh);
    const auto&                      elements = triMesh->getTrianglesVertices();
    const auto                       nV       = triMesh->getNumVertices();
    std::vector<std::vector<size_t>> onering(nV);

    for (size_t k = 0; k < elements.size(); ++k)
    {
        auto& tri = elements[k];
        onering[tri[0]].push_back(k);
        onering[tri[1]].push_back(k);
        onering[tri[2]].push_back(k);
    }

    std::vector<std::vector<bool>> E(nV, std::vector<bool>(nV, 1));

    auto addConstraint =
        [&](std::vector<size_t>& r1, std::vector<size_t>& r2,
            const size_t k, size_t i1, size_t i2)
        {
            if (i1 > i2) // Make sure i1 is always smaller than i2
            {
                std::swap(i1, i2);
            }
            if (E[i1][i2])
            {
                E[i1][i2] = 0;

                std::vector<size_t> rs(2);
                auto                it = std::set_intersection(r1.begin(), r1.end(), r2.begin(), r2.end(), rs.begin());
                rs.resize(static_cast<size_t>(it - rs.begin()));
                if (rs.size() > 1)
                {
                    size_t      idx = (rs[0] == k) ? 1 : 0;
                    const auto& tri = elements[rs[idx]];
                    for (size_t i = 0; i < 3; ++i)
                    {
                        if (tri[i] != tri[0] && tri[i] != tri[1])
                        {
                            idx = i;
                            break;
                        }
                    }
                    auto c = std::make_shared<PbdDihedralConstraint>();
                    c->initConstraint(*this, tri[2], tri[idx], tri[0], tri[1], stiffness);
                    m_constraints.push_back(std::move(c));
                }
            }
        };

    for (size_t k = 0; k < elements.size(); ++k)
    {
        auto& tri = elements[k];

        auto& r0 = onering[tri[0]];
        auto& r1 = onering[tri[1]];
        auto& r2 = onering[tri[2]];

        std::sort(r0.begin(), r0.end());
        std::sort(r1.begin(), r1.end());
        std::sort(r2.begin(), r2.end());

        addConstraint(r0, r1, k, tri[0], tri[1]);
        addConstraint(r0, r2, k, tri[0], tri[2]);
        addConstraint(r1, r2, k, tri[1], tri[2]);
    }
    return true;
}

bool
PbdModel::initializeConstantDensityConstraint(const double stiffness)
{
    // check if constraint type matches the mesh type
    if (m_mesh->getType() != Geometry::Type::SurfaceMesh
        && m_mesh->getType() != Geometry::Type::TetrahedralMesh
        && m_mesh->getType() != Geometry::Type::LineMesh
        && m_mesh->getType() != Geometry::Type::HexahedralMesh
        && m_mesh->getType() != Geometry::Type::PointSet)
    {
        //TODO: Really only need a point cloud, so may need to change this.
        LOG(WARNING) << "Constant constraint should come with a mesh";          
        return false;
    }

    auto c = std::make_shared<PbdConstantDensityConstraint>();
    c->initConstraint(*this, stiffness);
    m_constraints.push_back(std::move(c));

    return true;
}

void 
PbdModel::addDistanceConstraint(size_t v1, size_t v2, float stiffness)
{
	auto c = std::make_shared<PbdDistanceConstraint>();
	c->initConstraint(*this, v1, v2, stiffness);
	m_constraints.push_back(c);		
}

void 
PbdModel::addDihedralConstraint(size_t v0, size_t v1, size_t v2, size_t v3, float stiffness)
{
	auto c = std::make_shared<PbdDihedralConstraint>();
	c->initConstraint(*this, v0, v1, v2, v3, stiffness);
	m_constraints.push_back(c);
}

void
PbdModel::updateConstraintsFromCutting() 
{
	// As there is no smart way to query a constraint. Updating k constraints needs 
	// k * O(n) (search time) * O(n^2) (add per constraint time) = kO(n^3) 
	// which means we better call the reintilize function, which always takes O(n^3) independent of k
	//std::cout << "before updateConstraintsFromCutting, num constraints : "<< m_constraints.size() << std::endl;
	
	//reintilize constraints for cloth
	m_constraints.clear();
	this->initializeDistanceConstraints(m_stiffnessStreching);
	this->initializeDihedralConstraints(m_stiffnessBending);	

    //Add hard constraints on connected edges
    auto manager = std::dynamic_pointer_cast<SurfaceCuttingManager>(m_mesh);
    m_fixedVertexOnEdgeList.clear();
    if (manager)
    {
        for (auto& e : manager->m_brokenEdges)
        {
            if (e.isConnected )
            {
                if (e.nodeId[0] >= m_initialState->getPositions().size() || e.nodeId[1] >= m_initialState->getPositions().size())
                {
                    std::cout << "Error: can only hard constraint on initial edges...\n";
                    continue;
                }
                //Prevent the "gap" issue by adding hard constraints fixing the new vertices on partial broken edges
                std::tuple<size_t, size_t, size_t, float> tuple = std::make_tuple(e.newNodeId[0], e.nodeId[0], e.nodeId[1], e.brokenCoord-0.01);
                m_fixedVertexOnEdgeList.push_back(tuple);

                tuple = std::make_tuple(e.newNodeId[1], e.nodeId[0], e.nodeId[1], e.brokenCoord+0.01);
                m_fixedVertexOnEdgeList.push_back(tuple);
            }
        }
    }
    //partitionCostraints();
}

bool 
PbdModel::needsToHandleCut()
{
    auto manager = std::dynamic_pointer_cast<SurfaceCuttingManager>(m_mesh);
    if (manager && manager->m_newCutGenerated)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void
PbdModel::partitionCostraints(const bool print)
{
    // Form the map { vertex : list_of_constraints_involve_vertex }
    std::unordered_map<size_t, std::vector<size_t>> vertexConstraints;
    for (size_t constrIdx = 0; constrIdx < m_constraints.size(); ++constrIdx)
    {
        const auto& constr = m_constraints[constrIdx];
        for (const auto& vIds : constr->getVertexIds())
        {
            vertexConstraints[vIds].push_back(constrIdx);
        }
    }

    // Add edges to the constraint graph
    // Each edge represent a shared vertex between two constraints
    Graph constraintGraph(m_constraints.size());
    for (const auto& kv : vertexConstraints)
    {
        const auto& constraints = kv.second;     // the list of constraints for a vertex
        for (size_t i = 0; i < constraints.size(); ++i)
        {
            for (size_t j = i + 1; j < constraints.size(); ++j)
            {
                constraintGraph.addEdge(constraints[i], constraints[j]);
            }
        }
    }
    vertexConstraints.clear();

    // do graph coloring for the constraint graph
    const auto  coloring         = constraintGraph.doColoring();
    const auto& partitionIndices = coloring.first;
    const auto  numPartitions    = coloring.second;
    assert(partitionIndices.size() == m_constraints.size());

    m_partitionedConstraints.resize(0);
    m_partitionedConstraints.resize(static_cast<size_t>(numPartitions));

    for (size_t constrIdx = 0; constrIdx < partitionIndices.size(); ++constrIdx)
    {
        const auto partitionIdx = partitionIndices[constrIdx];
        m_partitionedConstraints[partitionIdx].push_back(std::move(m_constraints[constrIdx]));
    }

    // If a partition has size smaller than the partition threshold, then move its constraints back
    // These constraints will be processed sequentially
    // Because small size partitions yield bad performance upon running in parallel
    m_constraints.resize(0);
    for (const auto& constraints : m_partitionedConstraints)
    {
        if (constraints.size() < m_partitionThreshold)
        {
            for (size_t constrIdx = 0; constrIdx < constraints.size(); ++constrIdx)
            {
                m_constraints.push_back(std::move(constraints[constrIdx]));
            }
        }
    }

    // Remove all empty partitions
    size_t writeIdx = 0;
    for (size_t readIdx = 0; readIdx < m_partitionedConstraints.size(); ++readIdx)
    {
        if (m_partitionedConstraints[readIdx].size() >= m_partitionThreshold)
        {
            m_partitionedConstraints[writeIdx++] = std::move(m_partitionedConstraints[readIdx]);
        }
    }
    m_partitionedConstraints.resize(writeIdx);

    // Print
    if (print)
    {
        size_t numConstraints = 0;
        int    idx = 0;
        for (const auto& constraints : m_partitionedConstraints)
        {
            std::cout << "Partition # " << idx++ << " | # nodes: " << constraints.size() << std::endl;
            numConstraints += constraints.size();
        }
        std::cout << "Sequential processing # nodes: " << m_constraints.size() << std::endl;
        numConstraints += m_constraints.size();
        std::cout << "Total constraints: " << numConstraints << " | Graph size: "
                  << constraintGraph.size() << std::endl;
    }
}

void
PbdModel::handleCutting() 
{
	auto mesher = std::dynamic_pointer_cast<SurfaceCuttingManager>(m_mesh);
    mesher->m_newCutGenerated = false;
    mesher->propagateTopologyChanges();

	if (mesher && mesher->m_needToUpdatePhysicalModel)
	{	
        //std::cout << "PbdModel handle cutting ..." << std::endl;
		//update mass and invMass and fixedNodes
        auto numParticles = mesher->getVertexPositions().size();
        m_mass.resize(numParticles, 0);
        m_invMass.resize(numParticles, 0);
        setUniformMass(m_Parameters->m_uniformMassValue);
 

        for (auto i : m_Parameters->m_fixedNodeIds)
        {
            setFixedPoint(i);
        }

		//update PbdStates, note that the State vector length has changed
		m_currentState->setPositions(m_mesh->getVertexPositions());
		m_previousState->setPositions(m_mesh->getVertexPositions());

		auto& vel = m_currentState->getVelocities(); //Todo: Handle the new vertices' velocities and acelerations more realisticly?
		for (auto i = vel.size(); i < numParticles; i++)
			vel.push_back(Vec3d(0, 0, 0));

		auto& accn = m_currentState->getAccelerations();
		for (auto i = accn.size(); i < numParticles; i++)
			accn.push_back(Vec3d(0, 0, 0));

         //update initial state with new created vertices using the interpolation
        auto& interpolationData =  mesher->getNewVertexInterpolationData();
        auto &initPos = m_initialState->getPositions();
        auto numInitialParticles = initPos.size();
        for (auto i = numInitialParticles; i < numParticles; i++)
        {
            auto tuple = interpolationData[i - numInitialParticles];
            Vec3d interpolatedPos = initPos[std::get<0>(tuple)]*(1 - std::get<2>(tuple))
                + initPos[std::get<1>(tuple)]*std::get<2>(tuple);
            initPos.push_back(interpolatedPos);
        }
        interpolationData.clear();
		updateConstraintsFromCutting();
        mesher->m_needToUpdatePhysicalModel = false;
	}	
	
}

void
PbdModel::generateCut(std::shared_ptr<ToolState> info)
{
    auto manager = std::dynamic_pointer_cast<SurfaceCuttingManager>(m_mesh);
    manager->generateCut(info);
}


bool 
PbdModel::doGrasp(std::shared_ptr<ToolState> info)
{
    std::tuple<size_t, Vec3d*, Vec3d> tuple;
    auto manager = std::dynamic_pointer_cast<SurfaceCuttingManager>(m_mesh);
    if (manager->findToolSurfaceContactPair(info, tuple))
    {
        m_vertexPositionOffsetList.push_back(tuple);
        return true;
    }
    else
    {
        std::cout << "Error: can not find the vertex to grasp...\n";
        return false;
    }  
}

void
PbdModel::unGrasp(std::shared_ptr<ToolState> info)
{
    m_vertexPositionOffsetList.clear();
}

void
PbdModel::projectConstraints()
{	
    unsigned int i = 0;

    while (++i < m_Parameters->m_maxIter)
    {
        for (auto c: m_constraints)
        {
            c->solvePositionConstraint(*this);
        }

        for (auto& partitionConstraints : m_partitionedConstraints)
        {
            ParallelUtils::parallelFor(partitionConstraints.size(),
                [&](const size_t idx)
                {
                    partitionConstraints[idx]->solvePositionConstraint(*this);
                });
        }
    }
    ////TODO -- add hard attach constraint here
    if (m_fixedVertexOnEdgeList.size() > 0)
    {
        const auto& state = this->getCurrentState();
        auto& pos = state->getPositions();
        for (auto & tuple : m_fixedVertexOnEdgeList)
        {
            pos[std::get<0>(tuple)] += (1.0 - m_Parameters->m_viscousDampingCoeff) * 
                ( pos[std::get<1>(tuple)] * (1- std::get<3>(tuple)) + pos[std::get<2>(tuple)] * std::get<3>(tuple) - pos[std::get<0>(tuple)] );
        }
    }
    //picking (Grasping constraint) 
    if (m_vertexPositionOffsetList.size() > 0)
    {
        const auto& state = this->getCurrentState();
        auto& pos = state->getPositions();
        for (auto & tuple : m_vertexPositionOffsetList)
        {
            pos[std::get<0>(tuple)] = *std::get<1>(tuple) + std::get<2>(tuple);
        }      
    }
    
}

void
PbdModel::updatePhysicsGeometry()
{
    m_mesh->setVertexPositions(m_currentState->getPositions());
}

void
PbdModel::updatePbdStateFromPhysicsGeometry()
{
    m_currentState->setPositions(m_mesh->getVertexPositions());
}

void
PbdModel::setTimeStepSizeType(const TimeSteppingType type)
{
    m_timeStepSizeType = type;
    if (type == TimeSteppingType::fixed)
    {
        m_Parameters->m_dt = m_Parameters->m_DefaultDt;
    }
}

void
PbdModel::setUniformMass(const double val)
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
PbdModel::setParticleMass(const double val, const size_t idx)
{
    if (idx < m_mesh->getNumVertices())
    {
        m_mass[idx]    = val;
        m_invMass[idx] = 1.0 / val;
    }
}

void
PbdModel::setFixedPoint(const size_t idx)
{
    if (idx < m_mesh->getNumVertices())
    {
        m_invMass[idx] = 0;
    }
}

double
PbdModel::getInvMass(const size_t idx) const
{
    return m_invMass[idx];
}

void
PbdModel::integratePosition()
{
    const auto& accn    = m_currentState->getAccelerations();
    auto&       prevPos = m_previousState->getPositions();
    auto&       pos     = m_currentState->getPositions();
    auto&       vel     = m_currentState->getVelocities();

    ParallelUtils::parallelFor(m_mesh->getNumVertices(),
        [&](const size_t i)
        {
            if (std::abs(m_invMass[i]) > MIN_REAL)
            {
                vel[i] += (accn[i] + m_Parameters->m_gravity) * m_Parameters->m_dt;
                prevPos[i] = pos[i];
                pos[i] += (1.0 - m_Parameters->m_viscousDampingCoeff) * vel[i] * m_Parameters->m_dt;
            }
        });
}

void
PbdModel::updateVelocity()
{
    const auto& prevPos = m_previousState->getPositions();
    const auto& pos     = m_currentState->getPositions();
    auto&       vel     = m_currentState->getVelocities();

    ParallelUtils::parallelFor(m_mesh->getNumVertices(),
        [&](const size_t i)
        {
            if (std::abs(m_invMass[i]) > MIN_REAL)
            {
                vel[i] = (pos[i] - prevPos[i]) / m_Parameters->m_dt;
            }
        });
}
} // imstk
