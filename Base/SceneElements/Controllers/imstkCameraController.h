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

#ifndef imstkCameraController_h
#define imstkCameraController_h

#include "imstkModule.h"
#include "imstkTrackingController.h"
#include "imstkCamera.h"

#include <memory>

namespace imstk
{

///
/// \class CameraController
///
/// \brief
///
class CameraController : public Module, public TrackingController
{
public:
    ///
    /// \brief
    ///
    CameraController(std::string name, Camera& camera,
                     std::shared_ptr<DeviceClient> deviceClient = nullptr) :
        Module(name),
        m_camera(camera),
        TrackingController(deviceClient)
    {}

    ///
    /// \brief
    ///
	~CameraController() = default;

	///
	/// \brief
	///
    void setOffsetUsingCurrentCameraPose();

    ///
    /// \brief
    ///
    void setCameraRotationOffset(const Quatd& r);

    ///
    /// \brief
    ///
    void setCameraTranslationOffset(const Vec3d& t);

    ///
    /// \brief
    ///
    const Vec3d& getCameraTranslationOffset() const;

    ///
    /// \brief
    ///
    const Quatd& getCameraRotationOffset() const;

protected:
    ///
    /// \brief
    ///
	void initModule() override {};

    ///
    /// \brief
    ///
    void runModule() override;

    ///
    /// \brief
    ///
	void cleanUpModule() override {};

    Camera& m_camera; ///< Camera controlled by the external device

    Vec3d m_cameraTranslationOffset = WORLD_ORIGIN;     ///< Translation offset for the camera over tracking data
    Quatd m_cameraRotationOffset = Quatd::Identity();   ///< Rotation offset for the camera over tracking data

};

} // imstk
#endif // ifndef imstkCameraController_h