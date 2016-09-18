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

#include "imstkVegaMeshReader.h"
#include "imstkMeshReader.h"
#include "imstkTetrahedralMesh.h"
#include "imstkHexahedralMesh.h"

#include "g3log/g3log.hpp"

namespace imstk
{

std::shared_ptr<VolumetricMesh>
VegaMeshReader::read(const std::string& filePath, MeshFileType meshType)
{
    if (meshType != MeshFileType::VEG)
    {
        LOG(WARNING) << "VegaMeshReader::read error: file type not supported";
        return nullptr;
    }

    // Read Vega Mesh
    std::shared_ptr<vega::VolumetricMesh> vegaMesh = VegaMeshReader::readVegaMesh(filePath);

    // Convert to Volumetric Mesh
    return VegaMeshReader::convertVegaMeshToVolumetricMesh(vegaMesh);
}


std::shared_ptr<vega::VolumetricMesh>
VegaMeshReader::readVegaMesh(const std::string& filePath)
{
    auto fileName = const_cast<char*>(filePath.c_str());
    std::shared_ptr<vega::VolumetricMesh> vegaMesh(vega::VolumetricMeshLoader::load(fileName));
    return vegaMesh;
}

std::shared_ptr<imstk::VolumetricMesh>
VegaMeshReader::convertVegaMeshToVolumetricMesh(std::shared_ptr<vega::VolumetricMesh> vegaMesh)
{
    // Copy vertices
    std::vector<Vec3d> vertices;
    VegaMeshReader::copyVertices(vegaMesh, vertices);

    // Copy cells
    auto cellType = vegaMesh->getElementType();
    std::shared_ptr<imstk::VolumetricMesh> mesh;
    if(cellType == vega::VolumetricMesh::TET)
    {
        std::vector<TetrahedralMesh::TetraArray> cells;
        VegaMeshReader::copyCells<4>(vegaMesh, cells);

        auto tetMesh = std::make_shared<TetrahedralMesh>();
        tetMesh->initialize(vertices, cells, false);
        mesh = tetMesh;
    }
    else if(cellType == vega::VolumetricMesh::CUBIC)
    {
        std::vector<HexahedralMesh::HexaArray> cells;
        VegaMeshReader::copyCells<8>(vegaMesh, cells);

        auto hexMesh = std::make_shared<HexahedralMesh>();
        hexMesh->initialize(vertices, cells, false);
        mesh = hexMesh;
    }
    else
    {
        vegaMesh.reset();
        LOG(WARNING) << "VegaMeshReader::read error: invalid cell type.";
        return nullptr;
    }

    // Keep track of the vega mesh to initialize dynamical model
    mesh->setAttachedVegaMesh(vegaMesh);
    return mesh;
}

void
VegaMeshReader::copyVertices(std::shared_ptr<vega::VolumetricMesh> vegaMesh,
                             std::vector<Vec3d>& vertices)
{
    for(size_t i = 0; i < vegaMesh->getNumVertices(); ++i)
    {
        auto pos = *vegaMesh->getVertex(i);
        vertices.emplace_back(pos[0], pos[1], pos[2]);
    }
}

template<size_t dim>
void
VegaMeshReader::copyCells(std::shared_ptr<vega::VolumetricMesh> vegaMesh,
                          std::vector<std::array<size_t,dim>>& cells)
{
    std::array<size_t,dim> cell;
    for(size_t cellId = 0; cellId < vegaMesh->getNumElements(); ++cellId)
    {
        for (size_t i = 0; i < vegaMesh->getNumElementVertices(); ++i)
        {
            cell[i] = vegaMesh->getVertexIndex(cellId,i);
        }
        cells.emplace_back(cell);
    }
}

} // imstk
