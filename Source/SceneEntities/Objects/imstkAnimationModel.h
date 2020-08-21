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
#include "imstkSerialize.h"

namespace imstk
{
///
/// \class AnimationModel
///
/// \brief Contains geometric and animation render information
///
class AnimationModel
{
public:
    ///
    /// \brief Constructor
    ///
    explicit AnimationModel(std::shared_ptr<Geometry> geometry);

    AnimationModel() = delete;

    ///
    /// \brief Get/set geometry
    ///
    std::shared_ptr<Geometry> getGeometry();
    virtual void setGeometry(std::shared_ptr<Geometry> geometry);

    ///
    /// \brief Update animation
    ///
    virtual void update() {};

    ///
    /// \brief Reset animation
    ///
    virtual void reset() {};

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive, std::uint32_t const version)
    {
        UNUSED(version);
        archive(iMSTK_SERIALIZE(geometry));
    }
#endif

protected:
    friend class VulkanRenderer;
    friend class VTKRenderer;

    std::shared_ptr<Geometry> m_geometry = nullptr;
};
} // imstk
