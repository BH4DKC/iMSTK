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

#ifndef imstkIsometricMap_h
#define imstkIsometricMap_h

#include "imstkGeometryMap.h"

namespace imstk {

class IsometricMap : public GeometryMap
{
public:

    IsometricMap() :
        GeometryMap(GeometryMapType::Isometric),
        m_rigidTransform(RigidTransform3d::Identity())
    {}

    ~IsometricMap() = default;

    ///
    /// \brief Compute the map
    ///
    void compute() override {}

    ///
    /// \brief Apply the map
    ///
    void apply() override;

    ///
    /// \brief Check the validity of the map
    ///
    bool isValid() const { return true; };

    // Accessors
    void setTransform(const RigidTransform3d& affineTransform);
    const RigidTransform3d& getTransform() const;

protected:

    RigidTransform3d m_rigidTransform;

};
}

#endif // imstkIsometricMap_h
