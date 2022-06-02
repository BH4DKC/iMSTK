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

#include "ThreadInteraction.h"
#include "imstkLineMesh.h"
#include "imstkPbdObject.h"
#include "imstkCollisionDetectionAlgorithm.h"


#include "NeedleObject.h"

using namespace imstk;

ThreadInteraction::ThreadInteraction(std::shared_ptr<PbdObject>    tissueObj,
                                     std::shared_ptr<PbdObject>    threadObj)
    : PbdObjectCollision(tissueObj, threadObj)
{
    if (std::dynamic_pointer_cast<LineMesh>(threadObj->getCollidingGeometry()) == nullptr)
    {
        LOG(WARNING) << "ThreadInteraction only works with LineMesh collision geometry";
    }

    // Add collision handler for the PBD reaction
    //auto needlePbdCH = std::make_shared<NeedlePbdCH>();
    //needlePbdCH->setInputObjectA(tissueObj);
    //needlePbdCH->setInputObjectB(needleObj);
    // 
    // needlePbdCH->setThread(threadObj);
    // 
    //needlePbdCH->setInputCollisionData(getCollisionDetection()->getCollisionData());
    ////needlePbdCH->getCollisionSolver()->setCollisionIterations(1);
    //needlePbdCH->init();
    //setCollisionHandlingAB(needlePbdCH);


}
