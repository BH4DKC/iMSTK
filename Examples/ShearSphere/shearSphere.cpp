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
#include  "imstkVolumetricMesh.h"
#include "imstkMeshIO.h"

#include "vtkThinPlateSplineTransform.h"

using namespace imstk;

///
/// \brief This example demonstrates I/O of the mesh
///
int main()
{
    // SDK and Scene
    auto sdk = std::make_shared<SimulationManager>();
    auto scene = sdk->createNewScene("ReadMesh");

    // Read surface mesh
    auto volMesh = MeshIO::read(iMSTK_DATA_ROOT"/sphere.vtk");

    // Extract surface mesh
    auto volumeMesh = std::dynamic_pointer_cast<VolumetricMesh>(volMesh);    

    // Create object and add to scene
    auto object = std::make_shared<VisualObject>("meshObject");
    object->setVisualGeometry(volumeMesh);
    scene->addSceneObject(object);

    double maxy = -imstk::MAX_D;
    double miny = imstk::MAX_D;
    for (auto p : volumeMesh->getVertexPositions())
    {
        if (p.y()<miny)
            miny = p.y();

        if (p.y() > maxy)
            maxy = p.y();
    }

    ofstream file;
    file.open("deformation.txt");

    double range = maxy - miny;
    for (int i=0;i<volumeMesh->getNumVertices();++i)
    {
        Vec3d xx = volumeMesh->getInitialVertexPosition(i);
        xx.x() += range*(xx.y() - miny) / range;

        file << range*(xx.y() - miny) / range << "\n" << 0.0 << "\n" << 0.0 << "\n";

        volumeMesh->setVertexPosition(i, xx);
    }
    file.close();

    // generate thin plate spline transform
    auto p1 = vtkSmartPointer< vtkPoints >::New();
    p1->SetNumberOfPoints(volumeMesh->getNumVertices()/100);
    for (int i = 0; i < volumeMesh->getNumVertices()/100; ++i)
    {
        Vec3d initPos = volumeMesh->getInitialVertexPosition(i);
        p1->SetPoint(i, initPos.x(), initPos.y(), initPos.z());
    }

    auto p2 = vtkSmartPointer< vtkPoints >::New();
    p2->SetNumberOfPoints(volumeMesh->getNumVertices()/100);
    for (int i = 0; i < volumeMesh->getNumVertices()/100; ++i)
    {
        Vec3d defPos = volumeMesh->getVertexPosition(i);
        p2->SetPoint(i, defPos.x(), defPos.y(), defPos.z());
    }
    
    auto transform = vtkSmartPointer< vtkThinPlateSplineTransform >::New();
    transform->SetSourceLandmarks(p2);
    transform->SetTargetLandmarks(p1);
    transform->SetBasisToR();
    // You must invert the transform before passing it to vtkImageReslice
    transform->Inverse();
    //transform->InternalUpdate();

    /*double** weights = transform->getMatrix();

    ofstream fileWeights;
    fileWeights.open("weights.txt");
    fileWeights << "Num. of rows: " << volumeMesh->getNumVertices() + 4 << "\n";
    for (int i = 0; i < volumeMesh->getNumVertices()/100+4; ++i)
    {
        fileWeights << weights[i][0] << "\t" << weights[i][1] << "\t" << weights[i][2] << "\n";
    }
    fileWeights.close();*/

    ofstream fileTransforms;
    fileTransforms.open("transforms.txt");
    transform->PrintSelf(fileTransforms, vtkIndent(2));     
    fileTransforms.close();

    // Light
    auto light = std::make_shared<DirectionalLight>("light");
    light->setFocalPoint(Vec3d(5, -8, -5));
    light->setIntensity(1);
    scene->addLight(light);

    // Run
    sdk->setActiveScene(scene);
    sdk->startSimulation(SimulationStatus::PAUSED);

    return 0;
}
