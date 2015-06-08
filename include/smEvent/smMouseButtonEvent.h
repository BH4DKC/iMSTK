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

#ifndef SMMOUSEBUTTONEVENT_H
#define SMMOUSEBUTTONEVENT_H

// SimMedTK includes
#include "smEvent/smEvent.h"
#include "smCore/smMouse.h"
#include "smUtilities/smVector.h"

namespace smtk {
namespace Event {

class smMouseButtonEvent : public smEvent
{
public:
    static EventType EventName;

public:
    smMouseButtonEvent(const smMouseButton &button);

    const smMouseButton &getMouseButton();

    void setPresed(const bool &press);

    const bool &getPresed();

    const bool &togglePressed();

    void setWindowCoord(const smVec2d &coordinates);

    const smVec2d &getWindowCoord();

private:
    bool pressed; // If the button was pressed or released in this event
    smMouseButton mouseButton; // Which mouse button was pressed
    smVec2d coord; // X,Y coorindate relative to left edge
};

} // Event namespace
} // smtk namespace

#endif // SMMOUSEBUTTONEVENT_H