// This file is part of the SimMedTK project.
// Copyright (c) Center for Modeling, Simulation, and Imaging in Medicine,
//                        Rensselaer Polytechnic Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//---------------------------------------------------------------------------
//
// Authors:
//
// Contact:
//---------------------------------------------------------------------------

#include "KeyPressSDKShutdown.h"

#include "Core/SDK.h"
#include "Event/KeyboardEvent.h"

namespace mstk {
namespace Examples {
namespace Common {

KeyPressSDKShutdown::KeyPressSDKShutdown() : key(event::Key::Escape)
{
}

void KeyPressSDKShutdown::handleEvent(std::shared_ptr<core::Event> event)
{
    auto keyboardEvent = std::static_pointer_cast<event::KeyboardEvent>(event);
    if(keyboardEvent->getPressed())
    {
        if (keyboardEvent->getKeyPressed() == this->key)
        {
            SDK::getInstance()->shutDown();
        }
    }
}

void KeyPressSDKShutdown::setKey(event::Key key)
{
    this->key = key;
}

}//Common
}//Examples
}//tk