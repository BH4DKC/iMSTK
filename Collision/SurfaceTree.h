// This file is part of the SimMedTK project.
// Copyright (c) Center for Modeling, Simulation, and Imaging in Medicine,
//                        Rensselaer Polytechnic Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//---------------------------------------------------------------------------
//
// Authors:
//
// Contact:
//---------------------------------------------------------------------------

#ifndef SM_SMSURFACETREE_H
#define SM_SMSURFACETREE_H

// STD includes
#include <vector>
#include <array>

// SimMedTK includes
#include "Core/CollisionModelIterator.h"
#include "Collision/SurfaceTreeCell.h"
#include "Collision/CollisionMoller.h"
#include "Mesh/SurfaceMesh.h"

class MeshCollisionModel;

/// \brief !!
template<typename CellType>
class SurfaceTree : public CoreClass
{
protected:
  typedef Matrix44d MatrixType;

protected:
    int minTreeRenderLevel; 						///< !!
    bool renderSurface; 							///< !!
    bool enableShiftPos; 							///< !!
    bool enableTrianglePos; 						///< !!
    bool renderOnlySurface; 						///< true if rendering only the surface
    int totalCells; 								///< number of total cells
    std::vector<std::array<int,2>> levelStartIndex; ///< Stores levels start and end indices
    int currentLevel; ///<
    std::shared_ptr<MeshCollisionModel> model;

public:
    /// \brief constructor
    SurfaceTree(std::shared_ptr<MeshCollisionModel> surfaceModel, int maxLevels = 6);

    /// \brief destructor
    ~SurfaceTree();

    MatrixType transRot; ///< matrix for translation and rotation
    int maxLevel; ///< max level of the tree
    std::shared_ptr<CellType> root; ///< !!
    float shiftScale; ///< !!

    std::vector<CellType> initialTreeAllLevels; ///< !!
    std::vector<CellType> treeAllLevels; ///< !!

protected:

    /// \brief creates the tree based on input triangles
    bool createTree(std::shared_ptr<CellType> Node,
                    const std::vector<int> &triangles,
                    int siblingIndex);

public:

    /// \brief initialize the surface tree structure
    virtual void initStructure();

    virtual CollisionModelIterator<CellType>  getLevelIterator(int level) ;

    virtual CollisionModelIterator<CellType>  getLevelIterator() ;

    /// \brief !!
    inline std::shared_ptr<UnifiedId> getAttachedMeshID()
    {
        return this->model->getMesh()->getUniqueId();
    }

    /// \brief !!
    void handleEvent(std::shared_ptr<core::Event> p_event) override;

    /// \brief !! SurfaceTree structure
    void updateStructure();

    /// \brief !!
    void translateRot();

    std::shared_ptr<CellType> getRoot()
    {
        return root;
    }

    std::vector<std::pair<std::shared_ptr<CellType>,std::shared_ptr<CellType>>>
    getIntersectingNodes(std::shared_ptr<SurfaceTree<CellType>> otherTree)
    {
        std::vector<std::pair<std::shared_ptr<CellType>,std::shared_ptr<CellType>>> intersectingNodes;
        getIntersectingNodes(root, otherTree->getRoot(),intersectingNodes);

        return intersectingNodes;
    }

    void getIntersectingNodes(const std::shared_ptr<CellType> left,
                              const std::shared_ptr<CellType> right,
                              std::vector<std::pair<std::shared_ptr<CellType>,std::shared_ptr<CellType>>> &result );
};

#include "Collision/SurfaceTree.hpp"

#endif