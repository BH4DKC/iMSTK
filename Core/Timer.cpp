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

// SimMedTK includes
#include "Core/Timer.h"

Timer::Timer()
{
    start();
}
void Timer::start()
{
    begin = ClockType::now();
}
long double Timer::elapsed()
{
    long double deltaSec;
    TimePointType now = ClockType::now();
    DurationType delta = now - begin;
    deltaSec = ((static_cast<long double>(delta.count()) * PeriodType::num) / PeriodType::den);
    return deltaSec;
}