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

#include "imstkModule.h"
#include "imstkDeviceTracker.h"

namespace imstk
{
class Camera;

///
/// \class CameraController
///
/// \brief
///
class CameraController : public Module, public DeviceTracker
{
public:
    ///
    /// \brief TODO
    ///
    CameraController(
        std::shared_ptr<Camera> camera = nullptr,
        std::shared_ptr<DeviceClient> deviceClient = nullptr
    );

    ///
    /// \brief
    ///
    virtual ~CameraController() = default;

    ///
    /// \brief Set the offsets based on the current camera pose
    ///
    void setOffsetUsingCurrentCameraPose();

    ///
    /// \brief Get/Set translation offset of the camera
    ///
    const Vec3d& getCameraTranslationOffset() const;
    void setCameraTranslationOffset(const Vec3d& t);

    ///
    /// \brief Get/Set rotation offset of the camera
    ///
    const Quatd& getCameraRotationOffset() const;
    void setCameraRotationOffset(const Quatd& r);

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive)
    {
        archive(
            iMSTK_SERIALIZE_SUPERCLASS(Module),
            iMSTK_SERIALIZE_SUPERCLASS(DeviceTracker),
            iMSTK_SERIALIZE(camera),
            iMSTK_SERIALIZE(cameraTranslationOffset),
            iMSTK_SERIALIZE(cameraRotationalOffset)
        );
    }
#endif

protected:
    ///
    /// \brief TODO
    ///
    virtual void initModule() override {};

    ///
    /// \brief TODO
    ///
    virtual void runModule() override;

    ///
    /// \brief TODO
    ///
    void cleanUpModule() override {};

    std::shared_ptr<Camera> m_camera;     ///< Camera controlled by the external device

    Vec3d m_cameraTranslationOffset;      ///< Translation offset for the camera over tracking data
    Quatd m_cameraRotationalOffset;       ///< camera head angle offset (in deg)
};
} // imstk
