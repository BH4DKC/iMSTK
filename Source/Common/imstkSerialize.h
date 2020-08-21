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

#define UNUSED(x) (void)(x)

#ifdef iMSTK_ENABLE_SERIALIZATION

#include <cereal/access.hpp> // For LoadAndConstruct
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/atomic.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/list.hpp>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

namespace imstk {

#define iMSTK_SERIALIZE(name) \
 cereal::make_nvp<Archive>(#name, m_##name)

#define iMSTK_SERIALIZE_SUPERCLASS(className) \
 cereal::make_nvp<Archive>("imstk::"#className, cereal::base_class<##className>(this))

#define iMSTK_SERIALIZE_VIRTUAL_SUPERCLASS(className) \
 cereal::make_nvp<Archive>("imstk::"#className, cereal::virtual_base_class<##className>(this))

}

#define iMSTK_REGISTER_SERIALIZATION(className) \
 CEREAL_REGISTER_TYPE(##className)

#define iMSTK_VERSION(className, version) \
 CEREAL_CLASS_VERSION(##className, ##version)

#endif // iMSTK_ENABLE_SERIALIZATION
