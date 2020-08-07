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

#include "imstkDynamicObject.h"
#include "imstkSerialize.h"

namespace imstk
{
class SPHModel;

///
/// \class SPHObject
///
/// \brief Base class for scene objects that move and/or deform under
/// smooth particle hydrodynamics
///
class SPHObject : public DynamicObject
{
public:
    explicit SPHObject(const std::string& name);

    virtual ~SPHObject() override = default;

public:
    ///
    /// \brief Get the SPH model of the object
    ///
    std::shared_ptr<SPHModel> getSPHModel();

    ///
    /// \brief Initialize the SPH scene object
    ///
    bool initialize() override;

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive)
    {
        archive(
            iMSTK_SERIALIZE_SUPERCLASS(DynamicObject),
            iMSTK_SERIALIZE(SPHModel)
        );
    }
#endif

protected:
    std::shared_ptr<SPHModel> m_SPHModel = nullptr;
};
} // end namespace imstk
