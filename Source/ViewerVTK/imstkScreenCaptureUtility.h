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

#include <string>

namespace imstk
{
///
/// \class ScreenCaptureUtility
///
/// \brief Utility class to manage screen capture
///
class ScreenCaptureUtility
{
public:
    ///
    /// \brief Constructor
    ///
    ScreenCaptureUtility(std::string prefix = "Screenshot-");

    ///
    /// \brief Saves a screenshot with a name of <prefix><screenshotNumber>.<implementationImageType>
    ///        the <ImplementationImageType> is most likely `.png`.
    /// \return the file name that was actually used to store the file, "" if the file storage failed
    std::string saveScreenShot();

    ///
    /// \brief Saves a screenshot with the given name, the implementation will add the image
    ///        type used to store the file
    /// \param name base name for the file to store
    /// \return the file name that was actually used to store the file, "" if the file storage failed
    virtual std::string saveScreenShot(const std::string& name) = 0;

    ///
    /// \brief Returns the number of the next screenshot
    ///
    unsigned int getScreenShotNumber() const;

    ///
    /// \brief set/reset the prefix and the count numbers
    ///
    void setScreenShotPrefix(const std::string& newPrefix);

    ///
    /// \brief reset the screenshot number
    ///
    void resetScreenShotNumber();

protected:
    virtual ~ScreenCaptureUtility() = default;

    unsigned int m_screenShotNumber = 0; ///< screen shot number is added to the file prefix, and incremented everytime a screen shot is taken
    std::string  m_screenShotPrefix;     ///< the prefix for the screenshots to be saved
};
} // namespace imstk