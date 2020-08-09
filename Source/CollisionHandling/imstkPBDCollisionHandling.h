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

// imstk
#include "imstkSerialize.h"
#include "imstkCollisionHandling.h"

#include <vector>

namespace imstk
{
class PbdObject;
class PbdCollisionConstraint;
class PbdEdgeEdgeConstraint;
class PbdPointTriangleConstraint;
class PbdCollisionSolver;
struct CollisionData;

///
/// \class PBDCollisionHandling
///
/// \brief Implements PBD based collision handling
///
class PBDCollisionHandling : public CollisionHandling
{
public:

    ///
    /// \brief Constructor
    ///
    PBDCollisionHandling(const Side&                          side,
                         const std::shared_ptr<CollisionData> colData,
                         std::shared_ptr<PbdObject>           pbdObject1,
                         std::shared_ptr<PbdObject>           pbdObject2);

    PBDCollisionHandling() = default;

    ///
    /// \brief Compute forces based on collision data
    ///
    void processCollisionData() override;

    ///
    /// \brief Generate appropriate PBD constraints based on the collision data
    ///
    void generatePBDConstraints();

    std::shared_ptr<PbdCollisionSolver> getCollisionSolver() const { return m_pbdCollisionSolver; }

#ifdef iMSTK_ENABLE_SERIALIZATION
    ///
    /// \brief Serialization
    ///
    template<class Archive> void serialize(Archive & archive)
    {
        archive(
            iMSTK_SERIALIZE_SUPERCLASS(CollisionDetection),
            iMSTK_SERIALIZE(pbdObject1),
            iMSTK_SERIALIZE(pbdObject1),
            iMSTK_SERIALIZE(PBDConstraints),
            iMSTK_SERIALIZE(PBDSolver),
            iMSTK_SERIALIZE(EEConstraintPool),
            iMSTK_SERIALIZE(VTConstraintPool)
        );
    }
#endif

private:
    std::shared_ptr<PbdObject> m_PbdObject1 = nullptr; ///> PBD object
    std::shared_ptr<PbdObject> m_PbdObject2 = nullptr; ///> PBD object
    std::shared_ptr<PbdCollisionSolver> m_pbdCollisionSolver = nullptr;

    std::vector<std::shared_ptr<PbdCollisionConstraint>> m_PBDConstraints; ///> List of PBD constraints

    std::vector<std::shared_ptr<PbdEdgeEdgeConstraint>>      m_EEConstraintPool;
    std::vector<std::shared_ptr<PbdPointTriangleConstraint>> m_VTConstraintPool;
};
}
