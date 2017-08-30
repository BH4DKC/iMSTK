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

#ifndef imstkDynamicalModel_h
#define imstkDynamicalModel_h

#include <string>

#include "imstkVectorizedState.h"

namespace imstk
{
///
/// \brief Type of the time dependent mathematical model
///
enum class DynamicalModelType
{
    rigidBodyDynamics,
    elastoDynamics,
    positionBasedDynamics,
    NavierStokes,
    HeatEquation,
    none
};

///
/// \class DynamicalModel
///
/// \brief Base class for mathematical model of the physics governing the dynamic object
///
template <class StateType>
class DynamicalModel
{
public:

    ///
    /// \brief Type of the update of the state of the body
    ///
    enum class stateUpdateType
    {
        displacement,
        velocity,
        deltaDisplacement,
        deltaVelocity,
        none
    };

public:
    ///
    /// \brief Constructor
    ///
    DynamicalModel(DynamicalModelType type = DynamicalModelType::none) : m_type(type){}

    ///
    /// \brief Destructor
    ///
    virtual ~DynamicalModel() = default;

    ///
    /// \brief Return the initial state of the problem
    ///
    std::shared_ptr<StateType> getInitialState() { return m_initialState; }

    ///
    /// \brief Return the current state of the problem
    ///
    std::shared_ptr<StateType> getCurrentState() { return m_currentState; }

    ///
    /// \brief Return the previous state of the problem
    ///
    std::shared_ptr<StateType> getPreviousState() { return m_previousState; }

    ///
    /// \brief Reset the current state to the initial state
    ///
    virtual void resetToInitialState()
    {
        m_currentState->setState(m_initialState);
        m_previousState->setState(m_initialState);
    }

    ///
    /// \brief Returns the number of degrees of freedom
    ///
    std::size_t getNumDegreeOfFreedom() const { return m_numDOF; }
    void setNumDegreeOfFreedom(const size_t nDof) { m_numDOF = nDof; }

    ///
    /// \brief Get the type of the object
    ///
    const DynamicalModelType& getType() const { return m_type; }

    ///
    /// \brief Update states
    ///
    virtual void updateBodyStates(const Vectord& q, const stateUpdateType updateType = stateUpdateType::displacement) = 0;

    ///
    /// \brief
    ///
    virtual void updatePhysicsGeometry(){};

    ///
    /// \brief Initialize the dynamical model
    ///
    virtual bool initialize() = 0;

protected:

    DynamicalModelType m_type; ///> Mathematical model type

    // Body states
    std::shared_ptr<StateType> m_initialState;      ///> Initial state
    std::shared_ptr<StateType> m_currentState;      ///> Current state
    std::shared_ptr<StateType> m_previousState;     ///> Previous state

    std::size_t m_numDOF; ///> Total number of degree of freedom
};
} // imstk

#endif // ifndef imstkDynamicalModel_h