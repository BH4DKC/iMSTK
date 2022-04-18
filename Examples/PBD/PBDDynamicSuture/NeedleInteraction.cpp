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

#include "NeedleInteraction.h"
#include "imstkLineMesh.h"
#include "imstkPbdObject.h"
#include "imstkTaskGraph.h"
#include "imstkTetraToLineMeshCD.h"
// #include "NeedleEmbeddedCH.h"
// #include "NeedlePbdCH.h"
// #include "NeedleRigidBodyCH.h"
#include "imstkTetrahedralMesh.h"

#include "NeedleObject.h"

using namespace imstk;

NeedleInteraction::NeedleInteraction(std::shared_ptr<PbdObject>    tissueObj,
                                     std::shared_ptr<NeedleObject> needleObj) 
                                    : PbdObjectCollision(tissueObj, needleObj)
{
    if (std::dynamic_pointer_cast<LineMesh>(needleObj->getCollidingGeometry()) == nullptr)
    {
        LOG(WARNING) << "NeedleInteraction only works with LineMesh collision geometry on rigid NeedleObject";
    }
    if (std::dynamic_pointer_cast<TetrahedralMesh>(tissueObj->getPhysicsGeometry()) == nullptr)
    {
        LOG(WARNING) << "NeedleInteraction only works with TetrahedralMesh physics geometry on pbd tissueObj";
    }

}

// void
// NeedleInteraction::initGraphEdges(std::shared_ptr<TaskNode> source, std::shared_ptr<TaskNode> sink)
// {
//     PbdRigidObjectCollision::initGraphEdges(source, sink);
// }