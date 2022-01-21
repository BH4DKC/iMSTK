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

#include "imstkCollisionDetectionAlgorithm.h"

namespace imstk
{
///
/// \class UnidirectionalPlaneToSphereCD
///
/// \brief Plane to sphere collision detection
/// Generates point-direction contact data.
/// By default only generates contact data for the sphere
///
class UnidirectionalPlaneToSphereCD : public CollisionDetectionAlgorithm
{
public:
    UnidirectionalPlaneToSphereCD();
    virtual ~UnidirectionalPlaneToSphereCD() override = default;

    ///
    /// \brief Returns collision detection type string name
    ///
    virtual const std::string getTypeName() const override { return "UnidirectionalPlaneToSphereCD"; }

protected:
    ///
    /// \brief Compute collision data for AB simulatenously
    ///
    virtual void computeCollisionDataAB(
        std::shared_ptr<Geometry>      geomA,
        std::shared_ptr<Geometry>      geomB,
        std::vector<CollisionElement>& elementsA,
        std::vector<CollisionElement>& elementsB) override;
};
} // namespace imstk