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

#include "imstkSimulationManager.h"
#include "imstkPbdModel.h"
#include "imstkPbdObject.h"
#include "imstkAPIUtilities.h"
#include "imstkSurfaceMesh.h"
#include "imstkCollisionGraph.h"
#include "imstkCamera.h"
#include "imstkLight.h"
#include "imstkScene.h"
#include "imstkGeometry.h"

#include "imstkSerialize.h"

#include "cereal/archives/json.hpp"

#include <fstream>
#include <iostream>

using namespace imstk;

///
/// \brief \todo
///
int main()
{
    auto dynoObject = std::make_shared<DynamicObject>("Dyno");

    auto simManager = std::make_shared<SimulationManager>();
    auto scene      = simManager->createNewScene("PBDCloth");

    // Create surface mesh
    auto             surfMesh = std::make_shared<SurfaceMesh>();
    StdVectorOfVec3d vertList;
    const double     width  = 10.0;
    const double     height = 10.0;
    const int        nRows  = 16;
    const int        nCols  = 16;
    vertList.resize(nRows * nCols);
    const double dy = width / (double)(nCols - 1);
    const double dx = height / (double)(nRows - 1);
    for (int i = 0; i < nRows; ++i)
    {
        for (int j = 0; j < nCols; j++)
        {
            vertList[i * nCols + j] = Vec3d((double)dx * i, 1.0, (double)dy * j);
        }
    }
    surfMesh->setInitialVertexPositions(vertList);
    surfMesh->setVertexPositions(vertList);

    // Add connectivity data
    std::vector<SurfaceMesh::TriangleArray> triangles;
    for (std::size_t i = 0; i < nRows - 1; ++i)
    {
        for (std::size_t j = 0; j < nCols - 1; j++)
        {
            SurfaceMesh::TriangleArray tri[2];
            const size_t               index1 = i * nCols + j;
            const size_t               index2 = index1 + nCols;
            const size_t               index3 = index1 + 1;
            const size_t               index4 = index2 + 1;

            // Interleave [/][\]
            if (i % 2 ^ j % 2)
            {
                tri[0] = { { index1, index2, index3 } };
                tri[1] = { { index4, index3, index2 } };
            }
            else
            {
                tri[0] = { { index2, index4, index1 } };
                tri[1] = { { index4, index3, index1 } };
            }
            triangles.push_back(tri[0]);
            triangles.push_back(tri[1]);
        }
    }

    surfMesh->setTrianglesVertices(triangles);

    // Create Object & Model
    auto deformableObj = std::make_shared<PbdObject>("Cloth");
    auto pbdModel      = std::make_shared<PbdModel>();
    pbdModel->setModelGeometry(surfMesh);

    // configure model
    auto pbdParams = std::make_shared<PBDModelConfig>();

    // Constraints
    pbdParams->enableConstraint(PbdConstraint::Type::Distance, 0.1);
    pbdParams->enableConstraint(PbdConstraint::Type::Dihedral, 0.001);
    std::vector<size_t> fixedNodes(nCols);
    for (size_t i = 0; i < fixedNodes.size(); i++)
    {
        fixedNodes[i] = i;
    }
    pbdParams->m_fixedNodeIds = fixedNodes;

    // Other parameters
    pbdParams->m_uniformMassValue = 1.0;
    pbdParams->m_gravity   = Vec3d(0, -9.8, 0);
    pbdParams->m_DefaultDt = 0.005;
    pbdParams->m_maxIter   = 5;

    // Set the parameters
    pbdModel->configure(pbdParams);
    deformableObj->setDynamicalModel(pbdModel);
    deformableObj->setPhysicsGeometry(surfMesh);

    auto material = std::make_shared<RenderMaterial>();
    material->setBackFaceCulling(false);
    material->setColor(Color::LightGray);
    material->setDisplayMode(RenderMaterial::DisplayMode::WireframeSurface);
    auto surfMeshModel = std::make_shared<VisualModel>(surfMesh);
    surfMeshModel->setRenderMaterial(material);
    deformableObj->addVisualModel(surfMeshModel);

    // Light (white)
    auto whiteLight = std::make_shared<DirectionalLight>("whiteLight");
    whiteLight->setFocalPoint(Vec3d(5, -8, -5));
    whiteLight->setIntensity(7);

    // Light (red)
    auto colorLight = std::make_shared<SpotLight>("colorLight");
    colorLight->setPosition(Vec3d(-5, -3, 5));
    colorLight->setFocalPoint(Vec3d(0, -5, 5));
    colorLight->setIntensity(100);
    colorLight->setColor(Color::Red);
    colorLight->setSpotAngle(30);

    // Add in scene
    scene->addLight(whiteLight);
    scene->addLight(colorLight);
    scene->addSceneObject(deformableObj);

    auto debugPoints = std::make_shared<DebugRenderPoints>("Debug Points");
    auto debugMaterial = std::make_shared<RenderMaterial>();
    debugMaterial->setDebugColor(Color::Yellow);
    debugMaterial->setSphereGlyphSize(.01);
    debugPoints->setRenderMaterial(debugMaterial);
    scene->addDebugGeometry(debugPoints);

    scene->getCamera()->setFocalPoint(0, -5, 5);
    scene->getCamera()->setPosition(-15., -5.0, 15.0);

    // Serialize
    {
        std::ofstream os("out.cereal", std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        // Let there be light
        archive(scene);
    }

    // Deserialize
    std::unordered_map<std::string, std::shared_ptr<DirectionalLight> > newMap;
    auto newScene = simManager->createNewScene("deserialized_PBDCloth");
    {
        std::ifstream is("out.cereal", std::ios::binary);
        cereal::JSONInputArchive dearchive(is);

        dearchive(newScene);
    }

    simManager->setActiveScene(newScene);
    simManager->start(SimulationStatus::Paused);

    return 0;
}
