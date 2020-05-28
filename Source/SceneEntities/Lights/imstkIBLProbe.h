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

#include <memory>
#include <string>

#include "imstkTexture.h"
#include "imstkSerialize.h"

namespace imstk
{
class Texture;

///
/// \class IBLProbe
///
/// \brief Image-based lighting probe
///
/// Image-based lighting (IBL) probes are used to provide global illumination
/// using special cubemaps. The cubemaps are prefiltered and evaluated using a
/// lookup table (LUT) texture. The cubemaps should be preintegrated using
/// split-sum approximation.
///
class IBLProbe
{
public:
    ///
    /// Constructor
    ///
    /// \param irradianceCubemapPath path to .dds irradiance (diffuse) cubemap
    /// \param radianceCubemapPath path to .dds radiance (specular) cubemap
    /// \param brdfLUTPath path to BRDF LUT (shouldn't be .dds)
    ///
    IBLProbe(std::string irradianceCubemapPath,
             std::string radianceCubemapPath, std::string brdfLUTPath);

    /// \brief TODO
    std::shared_ptr<Texture> getIrradianceCubemapTexture();
    /// \brief TODO
    std::shared_ptr<Texture> getRadianceCubemapTexture();
    /// \brief TODO
    std::shared_ptr<Texture> getBrdfLUTTexture();

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive)
    {
        archive(
            iMSTK_SERIALIZE(irradianceCubemapPath),
            iMSTK_SERIALIZE(radianceCubemapPath),
            iMSTK_SERIALIZE(brdfLUTPath)
        );
    }

    template <class Archive>
    static void load_and_construct(Archive& archive, cereal::construct<IBLProbe>& construct)
    {
        std::string irradianceCubemapPath;
        std::string radianceCubemapPath;
        std::string brdfLUTPath;
        archive(
            irradianceCubemapPath,
            radianceCubemapPath,
            brdfLUTPath);
        construct(
            irradianceCubemapPath,
            radianceCubemapPath,
            brdfLUTPath);
    }
#endif

protected:
    std::string m_irradianceCubemapPath;
    std::string m_radianceCubemapPath;
    std::string m_brdfLUTPath;

    std::shared_ptr<Texture> m_irradianceCubemapTexture = nullptr;
    std::shared_ptr<Texture> m_radianceCubemapTexture   = nullptr;
    std::shared_ptr<Texture> m_brdfLUTTexture = nullptr;
};
}
