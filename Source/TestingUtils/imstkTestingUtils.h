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

#include "imstkSerialize.h"
#include "imstkCollisionHandling.h"

#include "gmock/gmock.h"

namespace imstk {

class TestWithTempFolder : public ::testing::Test
{
public:
    std::string getTempFolder();

protected:
    std::string m_tempFolder;

    ///
    /// \brief: Create tempFolder named based on the test in current folder.
    ///
    void SetUp() override;

    ///
    /// \brief: Delete tempFolder.
    ///
    void TearDown() override;
};

} // end namespace imstk
