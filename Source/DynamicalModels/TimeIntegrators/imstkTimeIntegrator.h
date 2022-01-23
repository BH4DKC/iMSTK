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

#include "imstkVectorizedState.h"

namespace imstk
{
///
/// \class TimeIntegrator
///
/// \brief This class defines the time integrators of various types.
/// It only sets the rules of how the velocity (or equivalent) and
/// acceleration (or equivalent) of the present time in terms of
/// positions (or equivalent) from previous time steps.
///
class TimeIntegrator
{
public:
    enum class Type
    {
        ForwardEuler,
        BackwardEuler,
        NewmarkBeta,
        CentralDifference,
        NoTimeStepper,
        None
    };

public:
    TimeIntegrator(Type type, double dT) : m_type(type), m_dT(dT), m_defaultDt(dT) {}
    virtual ~TimeIntegrator() = default;

    ///
    /// \brief Return the type of the time integrator
    ///
    TimeIntegrator::Type getType() const { return m_type; }

    ///
    /// \brief Get/Set the time step size
    ///@{
    void setTimestepSize(const double dT) { m_dT = dT; }
    double getTimestepSize() const { return m_dT; }
    void setTimestepSizeToDefault() { m_dT = m_defaultDt; }
    ///@}

    ///
    /// \brief Set/Get the time step size
    ///@{
    void setDefaultTimestepSize(const double dT) { m_defaultDt = dT; }
    double getDefaultTimestepSize() const { return m_defaultDt; }
    ///@}

    ///
    /// \brief Update states given the updates in different forms
    ///
    virtual void updateStateGivenDv(std::shared_ptr<FeDeformBodyState> prevState, std::shared_ptr<FeDeformBodyState> currentState, Vectord& dV) = 0;
    virtual void updateStateGivenDu(std::shared_ptr<FeDeformBodyState> prevState, std::shared_ptr<FeDeformBodyState> currentState, Vectord& dU) = 0;
    virtual void updateStateGivenV(std::shared_ptr<FeDeformBodyState> prevState, std::shared_ptr<FeDeformBodyState> currentState, Vectord& v)   = 0;
    virtual void updateStateGivenU(std::shared_ptr<FeDeformBodyState> prevState, std::shared_ptr<FeDeformBodyState> currentState, Vectord& u)   = 0;

protected:
    Type   m_type; ///> Type of the time integrator
    double m_dT;   ///> Delta T
    double m_defaultDt;
};
} // namespace imstk