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

#include "imstkAnalyticalGeometry.h"
#include "imstkSerialize.h"

namespace imstk
{
///
/// \class Cube
///
/// \brief Cube geometry
///
class Cube : public AnalyticalGeometry
{
public:
    explicit Cube(const Vec3d& pos = Vec3d(0.0, 0.0, 0.0), const double width = 1.0, const Vec3d& orientationAxis = Vec3d(0.0, 1.0, 0.0),
                  const std::string& name = std::string("")) : AnalyticalGeometry(Type::Cube, name)
    {
        setPosition(pos);
        setOrientationAxis(orientationAxis);
        setWidth(width);
    }

    ///
    /// \brief Print the cube info
    ///
    void print() const override;

    ///
    /// \brief Returns the volume of the cube
    ///
    double getVolume() const override;

    ///
    /// \brief Returns the width of the cube
    ///
    double getWidth(DataType type = DataType::PostTransform);

    ///
    /// \brief Sets the width of the cube
    ///
    void setWidth(const double w);

    ///
    /// \brief Returns signed distance to surface at pos
    /// \todo Doesn't support orientation yet
    ///
    double getFunctionValue(const Vec3d& pos) const override;

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive)
    {
        archive(
            iMSTK_SERIALIZE_SUPERCLASS(AnalyticalGeometry),
            iMSTK_SERIALIZE(width),
            iMSTK_SERIALIZE(widthPostTransform)
        );
    }
#endif

protected:
    friend class VTKCubeRenderDelegate;

    void applyScaling(const double s) override;
    void updatePostTransformData() const override;

    double m_width = 1.0;                      ///> Width of the cube
    mutable double m_widthPostTransform = 1.0; ///> Width of the cube once transform applied
};
}

#ifdef iMSTK_ENABLE_SERIALIZATION
CEREAL_REGISTER_TYPE(imstk::Cube)
#endif
