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

#include "imstkPbdConstraint.h"
#include "imstkSolverBase.h"
#include "imstkPbdCollisionConstraint.h"

namespace imstk
{
class PbdCollisionConstraint;

///
/// \struct CollisionConstraintData
/// \brief Stores positions and masses of two colliding objects
///
struct CollisionConstraintData
{
    CollisionConstraintData(std::shared_ptr<StdVectorOfVec3d> posA = nullptr,
                            std::shared_ptr<StdVectorOfReal>  invMassA = nullptr,
                            std::shared_ptr<StdVectorOfVec3d> posB = nullptr,
                            std::shared_ptr<StdVectorOfReal>  invMassB = nullptr) :
        m_posA(posA), m_invMassA(invMassA), m_posB(posB), m_invMassB(invMassB)
    {
    }

    std::shared_ptr<StdVectorOfVec3d> m_posA    = nullptr;
    std::shared_ptr<StdVectorOfReal> m_invMassA = nullptr;
    std::shared_ptr<StdVectorOfVec3d> m_posB    = nullptr;
    std::shared_ptr<StdVectorOfReal> m_invMassB = nullptr;

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive, std::uint32_t const version)
    {
        UNUSED(version);
        archive(
            iMSTK_SERIALIZE(posA),
            iMSTK_SERIALIZE(invMassA),
            iMSTK_SERIALIZE(posB),
            iMSTK_SERIALIZE(invMassB)
        );
    }
#endif
};

///
/// \class PbdSolver
///
/// \brief Position Based Dynamics solver
/// This solver can solve both partitioned constraints (unordered_set of vector'd constraints) in parallel
/// and sequentially on vector'd constraints. It requires a set of constraints, positions, and invMasses.
///
class PbdSolver : public SolverBase
{
public:
    typedef std::list<std::shared_ptr<std::vector<std::shared_ptr<PbdCollisionConstraint>>>> PbdCollisionList;

    ///
    /// \brief Constructors/Destructor
    ///
    PbdSolver();
    virtual ~PbdSolver() override = default;

    ///
    /// \brief Set Iterations. The number of nonlinear iterations.
    ///
    void setIterations(const size_t iterations) { this->m_iterations = iterations; }

    ///
    /// \brief Sets the partioned constraints the solver should solve for
    /// These will be solved in parallel
    ///
    void setPartitionedConstraints(std::shared_ptr<std::vector<PBDConstraintVector>> partitionedConstraints) { this->m_partitionedConstraints = partitionedConstraints; }

    ///
    /// \brief Sets the constraints the solver should solve for
    /// These wil be solved sequentially
    ///
    void setConstraints(std::shared_ptr<PBDConstraintVector> constraints) { this->m_constraints = constraints; }

    ///
    /// \brief Sets the positions the solver should solve with
    ///
    void setPositions(std::shared_ptr<StdVectorOfVec3d> positions) { this->m_positions = positions; }

    ///
    /// \brief Sets the invMasses the solver should solve with
    ///
    void setInvMasses(std::shared_ptr<StdVectorOfReal> invMasses) { this->m_invMasses = invMasses; }

    ///
    /// \brief Set time step
    ///
    void setTimeStep(const double dt) { m_dt = dt; }

    ///
    /// \brief Get Iterations. Returns current nonlinear iterations.
    ///
    size_t getIterations() const { return this->m_iterations; }

    ///
    /// \brief Set the PBD solver type
    ///
    void setSolverType(const PbdConstraint::SolverType& type);

    ///
    /// \brief Solve the non linear system of equations G(x)=0 using Newton's method.
    ///
    void solve() override;

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive, std::uint32_t const version)
    {
        UNUSED(version);
        archive(
            // No need to serialize SolverBase, it's empty
            iMSTK_SERIALIZE(iterations),
            iMSTK_SERIALIZE(dt),
            iMSTK_SERIALIZE(partitionedConstraints),
            iMSTK_SERIALIZE(constraints),
            iMSTK_SERIALIZE(positions),
            iMSTK_SERIALIZE(invMasses),
            iMSTK_SERIALIZE(solverType)
        );
    }
#endif

private:
    size_t m_iterations = 20;                                                             ///> Number of NL Gauss-Seidel iterations for regular constraints
    double m_dt;                                                                          ///> time step

    std::shared_ptr<std::vector<PBDConstraintVector>> m_partitionedConstraints = nullptr; ///> Set of vector'd/partitioned pbd constraints
    std::shared_ptr<PBDConstraintVector> m_constraints = nullptr;                         ///> Vector of constraints

    std::shared_ptr<StdVectorOfVec3d> m_positions = nullptr;
    std::shared_ptr<StdVectorOfReal>  m_invMasses = nullptr;
    PbdConstraint::SolverType m_solverType = PbdConstraint::SolverType::xPBD;
};

///
/// \class PbdCollisionSolver
///
/// \brief Position Based Dynamics collision solver
/// This solver can sequentially solve constraints in a list
///
class PbdCollisionSolver : SolverBase
{
public:
    PbdCollisionSolver();
    virtual ~PbdCollisionSolver() override = default;

    ///
    /// \brief Get CollisionIterations
    ///
    size_t getCollisionIterations() const { return this->m_collisionIterations; }

    ///
    /// \brief Add the global collision contraints to this solver
    ///
    void addCollisionConstraints(std::shared_ptr<PBDCollisionConstraintVector> constraints,
                                 std::shared_ptr<StdVectorOfVec3d> posA, std::shared_ptr<StdVectorOfReal> invMassA,
                                 std::shared_ptr<StdVectorOfVec3d> posB, std::shared_ptr<StdVectorOfReal> invMassB);

    ///
    /// \brief Solve the non linear system of equations G(x)=0 using Newton's method.
    ///
    void solve() override;

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    ///
    /// \brief Serialization
    ///
    template<class Archive> void save(Archive & archive, std::uint32_t const version) const
    {
        UNUSED(version);
        archive(
            // No need to serialize SolverBase, it's empty
            iMSTK_SERIALIZE(collisionIterations),
            iMSTK_SERIALIZE(collisionConstraints),
            iMSTK_SERIALIZE(collisionConstraintsData)
        );
    }
#endif

private:
    size_t m_collisionIterations = 5;                                                                   ///> Number of NL Gauss-Seidel iterations for collision constraints

    std::shared_ptr<std::list<std::shared_ptr<PBDCollisionConstraintVector>>> m_collisionConstraints = nullptr;///< Collision contraints charged to this solver
    std::shared_ptr<std::list<CollisionConstraintData>> m_collisionConstraintsData = nullptr;
};
} // imstk

#ifdef iMSTK_ENABLE_SERIALIZATION
iMSTK_REGISTER_SERIALIZATION(imstk::PbdCollisionSolver)
iMSTK_REGISTER_SERIALIZATION(imstk::PbdSolver)
#endif
