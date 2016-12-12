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

// imstk
#include "imstkInteractorStyle.h"
#include "imstkSimulationManager.h"
#include "imstkCameraController.h"
#include "imstkVirtualCouplingObject.h"

// vtk
#include "vtkObjectFactory.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkCamera.h"
#include "vtkAssemblyPath.h"
#include "vtkAbstractPropPicker.h"

// path
#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkLookupTable.h"
#include "vtkColorTransferFunction.h"
#include "vtkArrowSource.h"
#include "vtkGlyph3D.h"
#include "vtkProperty.h"
#include <string>

namespace imstk
{

vtkStandardNewMacro(InteractorStyle);
//
//bool 
//InteractorStyle::parseLogFileAndCalculateMetrics(std::string filename, vtkSmartPointer<vtkPoints> outPoints,
//    vtkSmartPointer<vtkFloatArray> outVelocities,
//    vtkSmartPointer<vtkFloatArray> outAccelerations,
//    vtkSmartPointer<vtkFloatArray> outJerks)
//{
//    // open logger file
//    fstream fstr(filename.c_str(), fstream::in);
//    if (!fstr.is_open() || !fstr.good() || fstr.eof())
//    {
//        LOG(WARNING) << "ERROR opening " << filename ;
//        return false;
//    }
//
//    // parse logger file
//    double min = MAX_D;
//    double max = MIN_D;
//    char line[1024];
//    while (fstr.good())
//    {
//        fstr.getline(line, 1024);
//        if (line[0] == '\0') continue;
//
//        char *ptr;
//        double x = 0.0, y = 0.0, z = 0.0;
//        int columnNumber = 0;
//        bool reTokenise = false;
//
//        if (strncmp(line, ",", 1) == 0)
//        {
//            ptr = nullptr;
//            reTokenise = true;
//        }
//        else
//        {
//            ptr = strtok(line, ",");
//        }
//
//        // Read columns
//        while (columnNumber < 4)
//        {
//            if (ptr != nullptr)
//            {
//                if (columnNumber == 1)
//                {
//                    x = atof(ptr);
//                }
//                else if (columnNumber == 2)
//                {
//                    y = atof(ptr);
//                }
//                else if (columnNumber == 3)
//                {
//                    z = atof(ptr);
//                }
//            }
//
//            if (reTokenise == false)
//            {
//                ptr = strtok(NULL, ",");
//            }
//            else
//            {
//                ptr = strtok(line, ",");
//                reTokenise = false;
//            }
//            columnNumber++;
//        }
//
//        // Add points or velocities
//        if (line[0] == 'P')
//        {
//            outPoints->InsertNextPoint(x, y, z);
//        }
//        else if (line[0] == 'V')
//        {
//            auto norm = Vec3d(x, y, z).norm();
//            outVelocities->InsertNextValue(norm);
//        }
//    }
//
//    auto numpts = outPoints->GetNumberOfPoints();
//    auto numdata = outVelocities->GetNumberOfValues();
//    if (numpts != numdata)
//    {
//        LOG(WARNING) << "ERROR reading " << filename << " : inconsistant number of points & velocity "
//            << "(" << numpts << " & " << numdata << ")";
//        return false;
//    }
//
//    //Calculate accelerations and jerks for each point
//    for (auto index = 0; index < numpts; index++)
//    {
//
//        if (index < 2 || index >= numpts - 2)
//        {
//            //Ignoring the boundary points
//            outJerks->InsertNextTuple3(0., 0., 0.);
//            if (index == 0 || index == numpts - 1)
//                outAccelerations->InsertNextTuple3(0., 0., 0.);
//            continue;
//        }
//
//		    Vec3d x(0., 0., 0.);
//		    Vec3d x_min1(0., 0., 0.);
//		    Vec3d x_min2(0., 0., 0.);
//		    Vec3d x_plus1(0., 0., 0.);
//		    Vec3d x_plus2(0., 0., 0.);
//
//        outPoints->GetPoint(index, x.data());
//
//
//        auto newInd = index - 1;
//        outPoints->GetPoint(newInd, x_min1.data());
//
//        newInd = index - 2;
//        outPoints->GetPoint(newInd, x_min2.data());
//
//        newInd = index + 1;
//        outPoints->GetPoint(newInd, x_plus1.data());
//
//        newInd = index + 2;
//        outPoints->GetPoint(newInd, x_plus2.data());
//        
//        //acceleration calculated as a second central derivative of position wrt time, h=1
//        auto acceleration = x_plus1 - 2 * x + x_min1;
//        //jerk calculated as the third derivative of position wrt time, h=1
//        auto jerk = 0.5*(x_plus2 - x_min2 - 2 * x_plus1 + 2 * x_min1);
//
//        LOG(INFO) << index << ", acceleration: " << acceleration << ", jerk: " << jerk;
//
//        outAccelerations->InsertNextTuple3(acceleration[0], acceleration[1], acceleration[2]);
//        outJerks->InsertNextTuple3(jerk[0], jerk[1], jerk[2]);
//    }
//
//    return true;
//}
//
//void
//InteractorStyle::displayPath(std::string deviceName)
//{
//    auto points = vtkSmartPointer<vtkPoints>::New();
//    auto dataArray = vtkSmartPointer<vtkFloatArray>::New();
//    dataArray->SetName("Velocity");
//    auto accelArray = vtkSmartPointer<vtkFloatArray>::New();
//    accelArray->SetName("Acceleration");
//    accelArray->SetNumberOfComponents(3);
//    auto jerkArray = vtkSmartPointer<vtkFloatArray>::New();
//    jerkArray->SetName("Jerk");
//    jerkArray->SetNumberOfComponents(3);
//    
//    // Precalculate all the metrics from device's log file
//    if (!parseLogFileAndCalculateMetrics(deviceName + ".log", points, dataArray, accelArray, jerkArray))
//        return;
//
//    // Set up spline points
//    auto polyData = vtkSmartPointer<vtkPolyData>::New();
//    polyData->SetPoints( points );
//    polyData->GetPointData()->AddArray( dataArray );
//    polyData->GetPointData()->SetActiveScalars("Velocity");
//
//    // Set up curves between adjacent points
//    auto numpts = points->GetNumberOfPoints();
//    polyData->Allocate( numpts-1 );
//    for (vtkIdType i = 0; i <  numpts-1; ++i)
//    {
//        vtkIdType ii[2];
//        ii[0] = i;
//        ii[1] = i+1;
//        polyData->InsertNextCell( VTK_LINE, 2, ii );
//    }
//
//    auto writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
//    writer->SetFileName((deviceName+".vtp").c_str());
//    writer->SetInputData(polyData);
//    writer->SetDataModeToAscii();
//    writer->Write();
//
//    // Lookup Table
//    double velocityRange[2];
//    dataArray->GetRange(velocityRange);
//    LOG(INFO) << "Velocity Norms range: " << velocityRange[0] << " " << velocityRange[1];
//    auto colors = vtkSmartPointer<vtkColorTransferFunction>::New();
//    colors->AddRGBPoint(velocityRange[0], 1, 0, 0);
//    colors->AddRGBPoint((velocityRange[1]-velocityRange[0])/3, 1, 0.5, 0);
//    colors->AddRGBPoint((velocityRange[1]-velocityRange[0])*2/3, 1, 1, 0);
//    colors->AddRGBPoint(velocityRange[1], 0, 1, 0);
//    colors->SetColorSpaceToRGB();
//
//    // Setup actor and mapper
//    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    mapper->SetInputData(polyData);
//    mapper->SetScalarRange(velocityRange[0], velocityRange[1]);
//    mapper->SetLookupTable(colors);
//    mapper->ScalarVisibilityOn();
//
//    auto actor = vtkSmartPointer<vtkActor>::New();
//    actor->SetMapper(mapper);
//
//
//    //Add actor for jerk glyphs
//    vtkSmartPointer<vtkArrowSource> arrow =
//        vtkSmartPointer<vtkArrowSource>::New();
//    polyData->GetPointData()->SetVectors(jerkArray);
//    vtkSmartPointer<vtkGlyph3D> glyphs =
//        vtkSmartPointer<vtkGlyph3D>::New();
//    glyphs->SetInputData(polyData);
//    glyphs->SetSourceConnection(arrow->GetOutputPort());
//    glyphs->ScalingOn();
//    glyphs->SetScaleModeToScaleByVector();
//    glyphs->SetColorModeToColorByVector();
//    glyphs->OrientOn();
//    glyphs->ClampingOff();
//    glyphs->SetVectorModeToUseVector();
//    glyphs->SetIndexModeToOff();
//
//    vtkSmartPointer<vtkPolyDataMapper> glyphMapper =
//        vtkSmartPointer<vtkPolyDataMapper>::New();
//    glyphMapper->SetInputConnection(glyphs->GetOutputPort());
//    glyphMapper->ScalarVisibilityOff();
//
//    vtkSmartPointer<vtkActor> glyphActor =
//        vtkSmartPointer<vtkActor>::New();
//    glyphActor->SetMapper(glyphMapper);
//    glyphActor->GetProperty()->SetColor(0., 0., 1.);
//
//    m_simManager->getViewer()->getCurrentRenderer()->getVtkRenderer()->AddActor(actor);
//    m_simManager->getViewer()->getCurrentRenderer()->getVtkRenderer()->AddActor(glyphActor);
//}

void
InteractorStyle::OnTimer()
{
    if (m_simManager->getStatus() != SimulationStatus::RUNNING)
    {
        return;
    }

    // Update Camera
    auto scene = m_simManager->getCurrentScene();
    m_simManager->getViewer()->getCurrentRenderer()->updateSceneCamera(scene->getCamera());

    // Update render delegates
    m_simManager->getViewer()->getCurrentRenderer()->updateRenderDelegates();

    // Reset camera clipping range
     m_simManager->getViewer()->getCurrentRenderer()->getVtkRenderer()->ResetCameraClippingRange();

    // Capture screen if triggered by the user
    if(m_simManager->getViewer()->shouldICaptureScreen())
    {
        m_simManager->getViewer()->captureScreen();
        m_simManager->getViewer()->setScreenCaptureState(false);
    }

    // Render
    this->Interactor->Render();
}

void
InteractorStyle::setOnCharEventHandler(vtkSlotFunctionType func, char c)
{
    m_charHandlerFunctionMap[c] = func;
}

void
InteractorStyle::OnChar()
{
    vtkRenderWindowInteractor *rwi = this->Interactor;

    char inputChar = rwi->GetKeyCode();

    if (m_charHandlerFunctionMap.find(inputChar) != m_charHandlerFunctionMap.end() &&
        !m_charHandlerFunctionMap[inputChar](this))
    {
        return;
    }
    
    switch (inputChar)
    {
    // Highlight picked actor
    case 'p' :
    case 'P' :
    {
        if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
        {
            return;
        }

        if(this->CurrentRenderer != 0)
        {
            if (this->State == VTKIS_NONE)
            {
                vtkAssemblyPath *path = nullptr;
                int *eventPos = rwi->GetEventPosition();
                this->FindPokedRenderer(eventPos[0], eventPos[1]);
                rwi->StartPickCallback();
                auto picker = vtkAbstractPropPicker::SafeDownCast(rwi->GetPicker());
                if ( picker != nullptr )
                {
                    picker->Pick(eventPos[0], eventPos[1], 0.0, this->CurrentRenderer);
                    path = picker->GetPath();
                }
                if ( path == nullptr )
                {
                    this->HighlightProp(nullptr);
                    this->PropPicked = 0;
                }
                else
                {
                    this->HighlightProp(path->GetFirstNode()->GetViewProp());
                    this->PropPicked = 1;
                }
                rwi->EndPickCallback();
            }
        }
        else
        {
            vtkWarningMacro(<<"no current renderer on the interactor style.");
        }
    }
    break;

    // Fly To picked actor
    case 'f' :
    case 'F' :
    {
        if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
        {
            return;
        }

        if(this->CurrentRenderer != 0)
        {
            this->AnimState = VTKIS_ANIM_ON;
            vtkAssemblyPath *path = nullptr;
            int *eventPos = rwi->GetEventPosition();
            this->FindPokedRenderer(eventPos[0], eventPos[1]);
            rwi->StartPickCallback();
            auto picker = vtkAbstractPropPicker::SafeDownCast(rwi->GetPicker());
            if ( picker != nullptr )
            {
                picker->Pick(eventPos[0], eventPos[1], 0.0, this->CurrentRenderer);
                path = picker->GetPath();
            }
            if (path != nullptr)
            {
                rwi->FlyTo(this->CurrentRenderer, picker->GetPickPosition());
            }
            this->AnimState = VTKIS_ANIM_OFF;
        }
        else
        {
            vtkWarningMacro(<<"no current renderer on the interactor style.");
        }
    }
    break;

    // Reset Camera
    case 'r' :
    case 'R' :
    {
        if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
        {
            return;
        }

        if(this->CurrentRenderer!=0)
        {
            this->CurrentRenderer->ResetCamera();
            this->CurrentRenderer->GetActiveCamera()->SetFocalPoint(0,0,0);
        }
        else
        {
            vtkWarningMacro(<<"no current renderer on the interactor style.");
        }
        rwi->Render();
    }
    break;

    // Stop Simulation
    case 's' :
    case 'S' :
    {
        m_simManager->startSimulation();
    }
    break;

    // Screen capture
    case 'b':
    case 'B':
    {
        if (m_simManager->getViewer()->isScreencaptureEnabled())
        {
            m_simManager->getViewer()->setScreenCaptureState(true);
        }
    }
    break;

    // Screen capture
    case 'v':
    case 'V':
    {
        LOG(INFO) << "=============Enable logging==============";
        auto camCtrl1 = m_simManager->getCurrentScene()->getCamera()->getController();
        const int logFrequency = camCtrl1->getLoggerFrequency();
        if(camCtrl1)
        {
            camCtrl1->enableLogging();
            camCtrl1->setLoggerFrequency(logFrequency);
        }

        auto virCoupObj = std::static_pointer_cast<VirtualCouplingObject>(m_simManager->getCurrentScene()->getSceneObject("tool"));
        if(virCoupObj)
        {
            virCoupObj->enableLogging();
            virCoupObj->setLoggerFrequency(logFrequency);
        }

    }
    break;

    case 'c':
    case 'C':
    {
        LOG(INFO) << "=============Disable logging==============";
        auto camCtrl1 = m_simManager->getCurrentScene()->getCamera()->getController();
        if(camCtrl1)
        {
            camCtrl1->disableLogging();
        }

        auto virCoupObj = std::static_pointer_cast<VirtualCouplingObject>(m_simManager->getCurrentScene()->getSceneObject("tool"));
        if(virCoupObj)
        {
            virCoupObj->disableLogging();
        }
    }
    break;

    // End Simulation
    case 'q' :
    case 'Q' :
    case 'e' :
    case 'E' :
    {
        LOG(INFO) << "=============End simulation==============";
        m_simManager->endSimulation();
    }
    break;

    // Play/Pause Simulation
    case ' ' :
    {
        if (m_simManager->getStatus() == SimulationStatus::RUNNING)
        {
            m_simManager->pauseSimulation();
        }
        else if (m_simManager->getStatus() == SimulationStatus::PAUSED)
        {
            m_simManager->runSimulation();
        }
    }
    break;

    // Quit Viewer
    case '\u001B':
    {
        m_simManager->getViewer()->endRenderingLoop();
    }break;

    }
}

void
InteractorStyle::OnMouseMove()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnMouseMove();
}

void
InteractorStyle::OnLeftButtonDown()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnLeftButtonDown();
}

void
InteractorStyle::OnLeftButtonUp()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnLeftButtonUp();
}

void
InteractorStyle::OnMiddleButtonDown()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnMiddleButtonDown();
}

void
InteractorStyle::OnMiddleButtonUp()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnMiddleButtonUp();
}

void
InteractorStyle::OnRightButtonDown()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnRightButtonDown();
}

void
InteractorStyle::OnRightButtonUp()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnRightButtonUp();
}

void
InteractorStyle::OnMouseWheelForward()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnMouseWheelForward();
}

void
InteractorStyle::OnMouseWheelBackward()
{
    if (m_simManager->getStatus() != SimulationStatus::INACTIVE)
    {
        return;
    }

    vtkBaseInteractorStyle::OnMouseWheelBackward();
}

void
InteractorStyle::setSimulationManager(SimulationManager *simManager)
{
    m_simManager = simManager;
}

} // imstk
