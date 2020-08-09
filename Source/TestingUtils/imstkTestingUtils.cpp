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

#include "imstkTestingUtils.h"

#include "vtkDirectory.h"

namespace imstk
{
std::string TestWithTempFolder::getTempFolder()
{
    return m_tempFolder;
}

void TestWithTempFolder::SetUp()
{
    m_tempFolder = std::string("./");
    m_tempFolder += ::testing::UnitTest::GetInstance()->current_test_info()->name();

    ASSERT_TRUE(vtkDirectory::MakeDirectory(m_tempFolder.c_str()));
}

void TestWithTempFolder::TearDown()
{
    ASSERT_TRUE(vtkDirectory::DeleteDirectory(m_tempFolder.c_str()));
}

}
