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
#include "imstkCamera.h"

#ifdef iMSTK_ENABLE_SERIALIZATION
#include <fstream>
#include "cereal/archives/json.hpp"
#endif

using namespace imstk;

class imstkCameraTest : public TestWithTempFolder
{
protected:
};

#ifdef iMSTK_ENABLE_SERIALIZATION
TEST_F(imstkCameraTest, Serialization)
{
    Camera cam;
    cam.setPosition(-3.0f, 0.0f, 50980129.120f);
    cam.setViewUp(-1.0f, 0.0001f, static_cast<float>(PI));
    cam.setFocalPoint(1.0f, 2.0f, static_cast<float>(PI));
    cam.setFieldOfView(98.0);

    // Serialize
    {
        std::ofstream os(getTempFolder() + "/imstkCamera.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(cam);
    }

    // Deserialize
    auto newCam = Camera();
    {
        std::ifstream is(getTempFolder() + "/imstkCamera.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newCam);
    }

    EXPECT_EQ(cam.getPosition(), newCam.getPosition());
    EXPECT_EQ(cam.getViewUp(), newCam.getViewUp());
    EXPECT_EQ(cam.getFocalPoint(), newCam.getFocalPoint());
    EXPECT_EQ(cam.getFieldOfView(), newCam.getFieldOfView());
}

#endif

int
imstkCameraTest(int argc, char* argv[])
{
    // Init Google Test & Mock
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests with gtest
    return RUN_ALL_TESTS();
}
