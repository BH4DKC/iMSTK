/*
 * // This file is part of the SimMedTK project.
 * // Copyright (c) Center for Modeling, Simulation, and Imaging in Medicine,
 * //                        Rensselaer Polytechnic Institute
 * //
 * // Licensed under the Apache License, Version 2.0 (the "License");
 * // you may not use this file except in compliance with the License.
 * // You may obtain a copy of the License at
 * //
 * //     http://www.apache.org/licenses/LICENSE-2.0
 * //
 * // Unless required by applicable law or agreed to in writing, software
 * // distributed under the License is distributed on an "AS IS" BASIS,
 * // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * // See the License for the specific language governing permissions and
 * // limitations under the License.
 * //
 * //---------------------------------------------------------------------------
 * //
 * // Authors:
 * //
 * // Contact:
 * //---------------------------------------------------------------------------
 */

#include "smEvent/smKeyboardEvent.h"

namespace smtk {
namespace Event {

EventType smKeyboardEvent::EventName = EventType::Keyboard;

smKeyboardEvent::smKeyboardEvent(const smKey& button)
    : pressed(false), key(button), modKey(smModKey::none)
{
}

const smKey& smKeyboardEvent::getKeyPressed()
{
    return key;
}
void smKeyboardEvent::setPressed(const bool& press)
{
    this->pressed = press;
}
const bool& smKeyboardEvent::getPressed()
{
    return this->pressed;
}
const bool& smKeyboardEvent::togglePressed()
{
    return this->pressed = !this->pressed;
}
void smKeyboardEvent::setModifierKey(const smModKey& modKey)
{
    this->modKey = modKey;
}
const smModKey& smKeyboardEvent::getModifierKey()
{
    return this->modKey;
}

} // Event namespace
} // smtk namespace