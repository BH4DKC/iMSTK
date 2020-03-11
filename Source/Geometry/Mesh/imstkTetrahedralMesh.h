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

#include <set>
#include <list>
#include <iostream>

// imstk
#include "imstkPointSet.h"
#include "imstkSurfaceMesh.h"
#include "imstkVolumetricMesh.h"

namespace imstk
{
///
/// \class TetrahedralMesh
///
/// \brief Tetrahedral mesh
///
class TetrahedralMesh : public VolumetricMesh
{
public:

    using TetraArray   = std::array<size_t, 4>;
    using WeightsArray = std::array<double, 4>;

    ///
    /// \brief Constructor
    ///
    TetrahedralMesh(const std::string name = std::string("")) : VolumetricMesh(Geometry::Type::TetrahedralMesh, name) {}

    ///
    /// \brief Initializes the rest of the data structures given vertex positions and
    ///  tetrahedra connectivity
    ///
    void initialize(const StdVectorOfVec3d&        vertices,
                    const std::vector<TetraArray>& tetrahedra,
                    bool                           computeAttachedSurfaceMesh = false);

    ///
    /// \brief Clear all the mesh data
    ///
    void clear() override;

    ///
    /// \brief Print the tetrahedral mesh
    ///
    void print() const override;

    ///
    /// \brief Compute and return the volume of the tetrahedral mesh
    ///
    double getVolume() const override;

    ///
    /// \brief Compute and set the attached surface mesh
    ///
    void computeAttachedSurfaceMesh() override;

    ///
    /// \brief This method
    /// (a) Extracts the confirming triangular mesh from the tetrahedral mesh
    /// (b) Checks and flips the triangle connectivity order if it is not consistent
    /// (c) Renumbers the vertices
    /// (d) optionally enforces the consistency of winding of resulting surface triangles
    bool extractSurfaceMesh(std::shared_ptr<SurfaceMesh> surfaceMesh, const bool enforceWindingConsistency = false);

    ///
    /// \brief compute the barycentric weights of a given point in 3D space for a given the tetrahedra
    ///
    void computeBarycentricWeights(const size_t& tetId, const Vec3d& pos, WeightsArray& weights) const;

    ///
    /// \brief Compute the bounding box of a given tetrahedron
    ///
    void computeTetrahedronBoundingBox(const size_t& tetId, Vec3d& min, Vec3d& max) const;

    // Accessors

    ///
    /// \brief set the vector of array of IDs for the mesh
    ///
    void setTetrahedraVertices(const std::vector<TetraArray>& tetrahedrons);

    ///
    /// \brief Return the vector of array of IDs for all the tetrahedra
    ///
    const std::vector<TetraArray>& getTetrahedraVertices() const;

    ///
    /// \brief Return the array of IDs for a given tetrahedron
    ///
    const TetraArray& getTetrahedronVertices(const size_t& tetId) const;

    ///
    /// \brief Returns the number of tetrahedra
    ///
    size_t getNumTetrahedra() const;

    ///
    /// \brief Get/set method for removed elements from the mesh
    ///
    void setTetrahedraAsRemoved(const unsigned int tetId) { m_removedMeshElems[tetId] = true; }
    const std::vector<bool>& getRemovedTetrahedra() const { return m_removedMeshElems; }

    ///
    /// \brief Create a tetrahedral mesh based on a uniform Cartesian mesh
    /// \param aabbMin  the small conner of a box
    /// \param aabbMax  the large conner of a box
    /// \param nx number of elements in the x-direction
    /// \param ny number of elements in the y-direction
    /// \param nz number of elements in the z-direction
    ///
    /// \note Refer: Dompierre, Julien & Labb�, Paul & Vallet, Marie-Gabrielle & Camarero, Ricardo. (1999).
    /// How to Subdivide Pyramids, Prisms, and Hexahedra into Tetrahedra.. 195-204.
    static std::shared_ptr<TetrahedralMesh> createUniformMesh(const Vec3d& aabbMin, const Vec3d& aabbMax, const size_t nx, const size_t ny, const size_t nz);

    ///
    /// \brief Create a tetrahedral mesh cover
    /// \note surfMesh can't be const since the non-const member function rayTracing is called inside.
    ///
    static std::shared_ptr<TetrahedralMesh> createTetrahedralMeshCover(std::shared_ptr<SurfaceMesh> surfMesh, const size_t nx, const size_t ny, size_t nz);
protected:

    friend class VTKTetrahedralMeshRenderDelegate;

    std::vector<TetraArray> m_tetrahedraVertices;///< vertices of the tetrahedra

    std::vector<bool> m_removedMeshElems;
};
}
