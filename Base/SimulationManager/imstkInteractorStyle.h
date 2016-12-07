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

#ifndef imstkInteractorStyle_h
#define imstkInteractorStyle_h

#include <memory>
#include <functional>
#include <unordered_map>

#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkPoints.h"
#include "vtkFloatArray.h"

namespace imstk
{

class SimulationManager;
class InteractorStyle;

using vtkBaseInteractorStyle = vtkInteractorStyleTrackballCamera;

///
/// \brief event handlers. This will return true if the default event handler behavior still needs to run, false otherwise
///
using vtkSlotFunctionType = std::function< bool(const InteractorStyle*) >;

///
/// \class InteractorStyle
///
/// \brief
///
class InteractorStyle : public vtkBaseInteractorStyle
{
public:

    static InteractorStyle *New();
    vtkTypeMacro(InteractorStyle, vtkBaseInteractorStyle);

    ///
    /// \brief
    ///
    virtual void OnTimer() override;

    ///
    /// \brief
    ///
    virtual void OnChar() override;

    ///
    /// \brief
    ///
    virtual void OnMouseMove() override;

    ///
    /// \brief
    ///
    virtual void OnLeftButtonDown() override;

    ///
    /// \brief
    ///
    virtual void OnLeftButtonUp() override;

    ///
    /// \brief
    ///
    virtual void OnMiddleButtonDown() override;

    ///
    /// \brief
    ///
    virtual void OnMiddleButtonUp() override;

    ///
    /// \brief
    ///
    virtual void OnRightButtonDown() override;

    ///
    /// \brief
    ///
    virtual void OnRightButtonUp() override;

    ///
    /// \brief
    ///
    virtual void OnMouseWheelForward() override;

    ///
    /// \brief
    ///
    virtual void OnMouseWheelBackward() override;

    ///
    /// \brief
    ///
    void setSimulationManager(SimulationManager* simManager);

    ///
    /// \brief
    ///
    void setOnCharEventHandler(vtkSlotFunctionType func, char c);

private:
    //functions
    bool parseLogFileAndCalculateMetrics(std::string filename, vtkSmartPointer<vtkPoints> outPoints,
        vtkSmartPointer<vtkFloatArray> outVelocities,
        vtkSmartPointer<vtkFloatArray> outAccelerations,
        vtkSmartPointer<vtkFloatArray> outJerks);
    void displayPath(std::string fileName);

    //map of OnCharHandler functions for each character
    std::unordered_map<char, vtkSlotFunctionType> m_charHandlerFunctionMap;
    SimulationManager* m_simManager;    ///>

};

} // imstk

#endif // ifndef imstkInteractorStyle_h
