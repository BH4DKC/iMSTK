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

#include "imstkGeometry.h"

namespace imstk
{
///
/// \class ImplicitGeometry
///
/// \brief Defines implicit geometry with an implicit function
/// Implicit functions must be decoupled from geometry
///
class ImplicitGeometry : public Geometry
{
public:
    ~ImplicitGeometry() override = default;

    ///
    /// \brief Returns function value given position
    ///
    virtual double getFunctionValue(const Vec3d& pos) const = 0;
};
} // namespace imstk
