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

#include "imstkVisualTest.h"

#include "imstkCamera.h"
#include "imstkCollisionDetectionVisualTest.h"
#include "imstkGeometryUtilities.h"
#include "imstkMeshToMeshBruteForceCD.h"
#include "imstkOrientedBox.h"
#include "imstkSurfaceMesh.h"
#include "imstkRigidObject2.h"
#include "imstkRigidBodyModel2.h"
#include "imstkRbdConstraint.h"
#include "imstkSphere.h"

#include "gtest/gtest.h"
#include "imstkScene.h"
#include "imstkPlane.h"
#include "imstkSceneManager.h"
#include "imstkRigidObjectCollision.h"
#include "imstkUnidirectionalPlaneToSphereCD.h"

using namespace imstk;

// NOTE move RigidBody struct into its own include file

class RigidCollisionHandlingVisualTest : public VisualTest
{

};

TEST_F(RigidCollisionHandlingVisualTest, UnidirectionalPlaneToSphere)
{
//     m_camera = std::make_shared<Camera>();
//     m_camera->setFocalPoint(-0.0366287, 0.420204, 0.474284);
//     m_camera->setPosition(-2.60143, 1.23713, 2.42823);
//     m_camera->setViewUp(0.216266, 0.968787, -0.121162);

    auto scene = std::make_shared<Scene>("Rigid Sphere w/ Plane");


    auto rbdModel = std::make_shared<RigidBodyModel2>();
    rbdModel->getConfig()->m_gravity = Vec3d(0.0, -9.81, 0.0);
    rbdModel->getConfig()->m_maxNumIterations = 15;

    auto sphere = std::make_shared<Sphere>(Vec3d(0,0,0),0.5);
    auto geom = GeometryUtils::toSurfaceMesh(sphere);
    //sphere->setPosition(0, 3, 0);

    auto obj1 = std::make_shared<RigidObject2>("Sphere");
    obj1->setVisualGeometry(sphere);
    obj1->setCollidingGeometry(sphere);
    obj1->setPhysicsGeometry(sphere);
    obj1->setDynamicalModel(rbdModel);
    obj1->getRigidBody()->m_mass = 1.0;
    obj1->getRigidBody()->m_initPos = Vec3d(0.0, 3.0, 0.0);
    //obj1->getRigidBody()->m_initOrientation = Quatd(Rotd(0.4, Vec3d(1.0, 0.0, 0.0)));
    obj1->getRigidBody()->m_intertiaTensor = Mat3d::Identity();

    scene->addSceneObject(obj1);

    auto plane = std::make_shared<Plane>(Vec3d(0.0, 0.0, 0.0), Vec3d(0.0, 1.0, 0.0));
    plane->setWidth(10.0);

    auto obj2 = std::make_shared<CollidingObject>("Plane");
    obj2->setVisualGeometry(plane);
    obj2->setCollidingGeometry(plane);
    scene->addSceneObject(obj2);
    
    scene->addInteraction(
        std::make_shared<RigidObjectCollision>(obj1, obj1, UnidirectionalPlaneToSphereCD::getStaticTypeName()));

    m_scene = scene;

    runFor(10.0);
}