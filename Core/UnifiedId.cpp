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

#include "Core/UnifiedId.h"

std::atomic_int UnifiedId::IDcounter;
UnifiedId::UnifiedId()
{
    ID = IDcounter.fetch_add(1);
}
bool UnifiedId::operator==(const UnifiedId* id)
{
    return (ID == id->getId());
}
bool UnifiedId::operator!=(const short int& id)
{
    return (ID != id);
}