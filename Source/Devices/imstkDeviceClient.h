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

#include "imstkMath.h"

namespace imstk
{
///
/// \class DeviceClient
///
/// \brief Base class for any device client, this provides a common interface
/// for other imstk classes to access quantities without knowing anything about
/// the api of the device
///
class DeviceClient
{
public:
    ///
    /// \brief Constructor
    ///
    DeviceClient(const std::string& name = "", const std::string& ip = "");

    ///
    /// \brief Destructor
    ///
    virtual ~DeviceClient() = default;

    ///
    /// \brief Get/Set the device IP
    ///
    const std::string& getIp();
    void setIp(const std::string& ip);

    ///
    /// \brief Get/Set the device name
    ///
    const std::string& getDeviceName();
    void setDeviceName(const std::string& deviceName);

    ///
    /// \brief Get/Set what listeners to enable on the device: tracking, analogic, force, buttons.
    ///
    const bool& getTrackingEnabled() const;
    void setTrackingEnabled(const bool& status);
    const bool& getAnalogicEnabled() const;
    void setAnalogicEnabled(const bool& status);
    const bool& getButtonsEnabled() const;
    void setButtonsEnabled(const bool& status);
    const bool& getForceEnabled() const;
    void setForceEnabled(const bool& status);

    ///
    /// \brief Get the device position
    ///
    const Vec3d& getPosition() const;

    ///
    /// \brief Get the device velocity
    ///
    const Vec3d& getVelocity() const;

    ///
    /// \brief Get the device orientation
    ///
    const Quatd& getOrientation() const;

    ///
    /// \brief Get the status of the device buttons
    ///
    const std::map<size_t, bool>& getButtons() const;

    ///
    /// \brief Get the status of a device button
    ///
    bool getButton(size_t buttonId) const;

    ///
    /// \brief Get/Set the device force
    ///
    const Vec3d& getForce() const;
    void setForce(Vec3d force);

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive)
    {
        archive(
            iMSTK_SERIALIZE(deviceName),
            iMSTK_SERIALIZE(ip),
            iMSTK_SERIALIZE(trackingEnabled),
            iMSTK_SERIALIZE(analogicEnabled),
            iMSTK_SERIALIZE(buttonsEnabled),
            iMSTK_SERIALIZE(forceEnabled),
            iMSTK_SERIALIZE(position),
            iMSTK_SERIALIZE(velocity),
            iMSTK_SERIALIZE(orientation),
            iMSTK_SERIALIZE(buttons),
            iMSTK_SERIALIZE(force)
        );
    }
#endif

protected:
    std::string m_deviceName;                ///< Device Name
    std::string m_ip;                        ///< Connection device IP

    bool m_trackingEnabled = true;           ///< Tracking enabled if true
    bool m_analogicEnabled = true;           ///< Analogic enabled if true
    bool m_buttonsEnabled  = true;           ///< Buttons enabled if true
    bool m_forceEnabled    = false;          ///< Force enabled if true

    Vec3d m_position;                        ///< Position of end effector
    Vec3d m_velocity;                        ///< Linear velocity of end effector
    Quatd m_orientation;                     ///< Orientation of the end effector
    std::map<size_t, bool> m_buttons;        ///< Buttons: true = pressed/false = not pressed
    Vec3d m_force;                           ///< Force vector
};
}
