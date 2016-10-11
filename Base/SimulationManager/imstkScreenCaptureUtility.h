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

#ifndef imstkScreenCaptureUtility_h
#define imstkScreenCaptureUtility_h

// Screenshot
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

namespace imstk
{

///
/// \class screenCaptureUtility
///
/// \brief Utility class to manage screen capture
///
class screenCaptureUtility
{
public:
    ///
    /// \brief Constructor
    ///
    screenCaptureUtility(const std::string prefix = "Screenshot-")
    {
        m_screenShotPrefix = prefix;
        auto m_windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
        auto m_pngWriter = vtkSmartPointer<vtkPNGWriter>::New();
    };

    ///
    /// \brief Destructor
    ///
    ~screenCaptureUtility() = default;

    ///
    /// \brief Saves the screenshot as a png file
    ///
    void saveScreenShot(vtkRenderWindow* rw)
    {
        if (m_windowToImageFilter->GetInput() == nullptr)
        {
            m_windowToImageFilter->SetInput(rw);

            m_windowToImageFilter->SetMagnification(1);
            m_windowToImageFilter->SetInputBufferTypeToRGB();
            m_windowToImageFilter->ReadFrontBufferOff();
            m_windowToImageFilter->Update();

            m_pngWriter->SetInputConnection(m_windowToImageFilter->GetOutputPort());
        }

        m_windowToImageFilter->Modified();

        std::string captureName = m_screenShotPrefix + std::to_string(m_screenShotNumber) + ".png";

        m_pngWriter->SetFileName(captureName.data());
        m_pngWriter->Write();

        std::cout << "Screen shot " << m_screenShotNumber << " saved as " << captureName << "\n";

        m_screenShotNumber++;
    };

    ///
    /// \brief Returns the number of the screenshot
    ///
    unsigned int getScreenShotNumber() const
    {
        return m_screenShotNumber;
    };

protected:
    vtkNew<vtkWindowToImageFilter> m_windowToImageFilter;
    vtkNew<vtkPNGWriter> m_pngWriter;
    unsigned int m_screenShotNumber = 0; //>
    std::string m_screenShotPrefix = "screenShot-"; //>
};

} // imstk

#endif // ifndef imstkScreenCaptureUtility_h