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

#include "imstkDeviceClient.h"

#include <unordered_map>

namespace imstk
{
using KeyStateType = int;
#define KEY_RELEASE 0
#define KEY_PRESS 1

///
/// \class KeyEvent
///
/// \brief Provides the information of a key event (press, release, & which key)
///
class KeyEvent : public Event
{
public:
    KeyEvent(const std::string type, const char key, const KeyStateType keyPressType) : Event(type), m_keyPressType(keyPressType), m_key(key) { }
    ~KeyEvent() override = default;

public:
    KeyStateType m_keyPressType;
    const char   m_key = -1;
};

///
/// \class KeyboardDeviceClient
///
/// \brief This class provides the keyboard state it also posts events
///
class KeyboardDeviceClient : public DeviceClient
{
protected:
    ///
    /// This object is only creatable through its New method
    ///
    KeyboardDeviceClient() : DeviceClient("KeyboardDevice", "") { }

    ///
    /// This object is only creatable through this method
    ///
    static std::shared_ptr<KeyboardDeviceClient> New();

public:
    ~KeyboardDeviceClient() override = default;

    // Only these classes may provide this object
    friend class VTKInteractorStyle;
    friend class ConsoleModule;

public:
    // *INDENT-OFF*
    ///
    /// \brief Posted when key is pressed (not continuously)
    ///
    SIGNAL(KeyboardDeviceClient, keyPress);

    ///
    /// \brief Posted when key is released (not continuously)
    ///
    SIGNAL(KeyboardDeviceClient, keyRelease);
    // *INDENT-ON*

public:
    const std::unordered_map<int, KeyStateType>& getKeyState() const { return m_buttons; }

protected:
    ///
    /// \brief Post a key press
    ///
    void emitKeyDown(char key);

    ///
    /// \brief Post a key release
    ///
    void emitKeyUp(char key);

    ///
    /// \brief Returns true if key is currently down
    ///
    bool isKeyDown(const char key) const;
};
} // namespace imstk