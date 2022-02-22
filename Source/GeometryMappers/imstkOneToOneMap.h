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

#include "imstkGeometryMap.h"
#include "imstkTypes.h"

namespace imstk
{
template<typename T, int N> class VecDataArray;
class PointSet;

///
/// \class OneToOneMap
///
/// \brief For each vertex on the child geometry OneToOneMap will try and find
/// a corresponding point on the parent geometry.
///
/// In the default configuration only points that _actually_ coincide will be
/// considered matches (eps = std::numeric_limits<double>::min())
/// On `apply()` all vertices in the child geometry that have corresponding matches
/// will be moved to the position of their corresponding point in the parent geometry
///
class OneToOneMap : public GeometryMap
{
public:
    OneToOneMap() = default;

    OneToOneMap(
        std::shared_ptr<Geometry> parent,
        std::shared_ptr<Geometry> child);

    const std::string getTypeName() const override { return "OneToOneMap"; }

    ///
    /// \brief Compute the tetra-triangle mesh map
    ///
    void compute() override;

    ///
    /// \brief Check the validity of the map
    ///
    bool isValid() const override;

    ///
    /// \brief Sets the one-to-one correspondence directly
    ///
    void setMap(const std::unordered_map<IndexType, IndexType>& sourceMap);

    ///
    /// \brief Apply (if active) the tetra-triangle mesh map
    ///
    void apply() override;

    ///
    /// \brief Print the map
    ///
    void print() const override;

    ///
    /// \brief Set the geometry that dictates the map
    ///
    void setParentGeometry(std::shared_ptr<Geometry> parent) override;

    ///
    /// \brief Set the geometry that follows the parent
    ///
    void setChildGeometry(std::shared_ptr<Geometry> child) override;

    ///
    /// \brief Get the corresponding parent index, given a child index
    /// \param index on the child geometry
    ///
    IndexType getMapIdx(const IndexType idx) const;

    ///
    /// \brief Set/Get the tolerance. The distance to consider
    /// two points equivalent/corresponding
    ///@{
    void setTolerance(const double tolerance) { m_epsilon = tolerance; }
    double getTolerance() const { return m_epsilon; }
///@}

protected:
    ///
    /// \brief Returns the first matching vertex
    ///
    IndexType findMatchingVertex(const VecDataArray<double, 3>& parentMesh, const Vec3d& p);

    std::unordered_map<IndexType, IndexType> m_oneToOneMap;   ///> One to one mapping data

    // This vector is for parallel processing, it should contain identical data as m_oneToOneMap
    std::vector<std::pair<IndexType, IndexType>> m_oneToOneMapVector; ///< One to one mapping data

    double m_epsilon = IMSTK_DOUBLE_EPS;                              ///< Tolerance for considering two points equivalent
};
} // namespace imstk