#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <iomanip>

// Objects
#include "imstkSceneObject.h"
#include "imstkVirtualCouplingObject.h"
#include "imstkLight.h"
#include "imstkCamera.h"

// Geometry
#include "imstkPlane.h"
#include "imstkTetrahedralMesh.h"
#include "imstkSurfaceMesh.h"
#include "imstkMeshReader.h"

// logger
#include "g3log/g3log.hpp"
#include "imstkUtils.h"

// testVTKTexture
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <string>
#include <vtkJPEGReader.h>

// Overlay
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <vtkImageResize.h>
#include <vtkImageTranslateExtent.h>
#include <vtkImageMapper.h>
#include <vtkActor2D.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkImageResize.h>
#include <vtkImageTranslateExtent.h>

// Screenshot
#include <vtkRenderWindowInteractor.h>
#include <vtkObjectFactory.h>
#include <vtkRendererCollection.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>

using namespace imstk;

// Texture coordinates
const imstk::Vec2d texCenter(750.0, 750.0);
const imstk::Vec2d texCircleTop(750.0, 250.0);
const imstk::Vec2d texCircleBottom(750.0, 1250.0);
const imstk::Vec2d texTopLeftCorner(252.0, 665.0);
const imstk::Vec2d texTopRightCorner(1254.0, 665.0);
const imstk::Vec2d texBottomRightCorner(1254.0, 832.0);
const imstk::Vec2d texBottomLeftCorner(252.0, 832.0);

template <typename T>
struct targetPoints
{
    T center, top, bottom;
    T corners[4];// in clockwise from top left
};

typedef targetPoints<imstk::Vec2d> screenSpacePoints;
typedef targetPoints<imstk::Vec3d> targetPointsInWorld;
typedef targetPoints<imstk::Vec3d> screenSpacePtsWithDepth;

targetPointsInWorld targetWorldPoints[6];


const std::string metricsFileNamePrefix = "cameraNavMetrics-";
const std::string screenShotPrefix = "screenShot-";

///
/// keep track of the screen capture
///
class screenCaptureUtil
{

    screenCaptureUtil(vtkRenderWindow* rw) : m_triggerScreenCapture(false), m_screenShotNumber(0)
    {
        m_windowToImageFilter->SetInput(rw);
        m_windowToImageFilter->SetMagnification(1);
        m_windowToImageFilter->SetInputBufferTypeToRGB();
        m_windowToImageFilter->ReadFrontBufferOff();
        m_windowToImageFilter->Update();

        m_pngWriter->SetInputConnection(m_windowToImageFilter->GetOutputPort());
    };

    ~screenCaptureUtil(){};

    void saveScreenShot()
    {
        m_windowToImageFilter->Modified();

        std::string captureName = screenShotPrefix + std::to_string(m_screenShotNumber) + ".png";

        m_pngWriter->SetFileName(captureName.data());
        m_pngWriter->Write();

        std::cout << "Screen shot " << m_screenShotNumber << " saved.\n";

        m_screenShotNumber++;
        m_triggerScreenCapture = false;
    };

    unsigned int getScreenShotNumber() const
    {
        return m_screenShotNumber;
    };

protected:
    vtkNew<vtkWindowToImageFilter> m_windowToImageFilter;
    vtkNew<vtkPNGWriter> m_pngWriter;
    bool m_triggerScreenCapture;
    unsigned int m_screenShotNumber;
};

///
///	 \brief Add a 2D overlay of target markers on a 3D scene
///
void add2DTextureOverlay(const vtkSmartPointer<vtkRenderer>& rendererVtk, const char* fileName, const int size)
{
    // Read the image
    vtkNew<vtkPNGReader> reader;
    reader->SetFileName(fileName);
    reader->Update();

    int dim[3] = { size, size, 1 };

    // Resize image
    vtkNew<vtkImageResize> resize;
    resize->SetInputConnection(reader->GetOutputPort());
    resize->SetOutputDimensions(dim);

    // Translate image extent (origin to its center)
    vtkNew<vtkImageTranslateExtent> translateExtent;
    translateExtent->SetInputConnection(resize->GetOutputPort());
    translateExtent->SetTranslation(-dim[0] / 2, -dim[1] / 2, 0);

    // Mapper
    vtkNew<vtkImageMapper> imageMapper;
    imageMapper->SetInputConnection(translateExtent->GetOutputPort());
    imageMapper->SetColorWindow(255);
    imageMapper->SetColorLevel(127);

    // Actor
    vtkNew<vtkActor2D> imageActor;
    imageActor->SetMapper(imageMapper.GetPointer());
    imageActor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedDisplay();
    imageActor->SetPosition(0.5, 0.5);

    // Renderer
    rendererVtk->AddActor2D(imageActor.GetPointer());
}

///
/// \brief Create a plane overlaid with a texture given four vertices
///
void createPlaneTargetWithTexture(
    const std::shared_ptr<imstk::Scene>& scene,
    const imstk::Vec3d& a,
    const imstk::Vec3d& b,
    const imstk::Vec3d& c,
    const imstk::Vec3d& d,
    const std::string& texFileName,
    const std::string& planeName)
{
    // Read surface mesh
    auto objMesh = imstk::MeshReader::read("Resources/plane.obj");

    objMesh->setInitialVerticePosition(0, a);
    objMesh->setInitialVerticePosition(1, b);
    objMesh->setInitialVerticePosition(2, c);
    objMesh->setInitialVerticePosition(3, d);

    auto surfaceMesh = std::dynamic_pointer_cast<imstk::SurfaceMesh>(objMesh);
    surfaceMesh->addTexture(texFileName);

    // Create object and add to scene
    auto object = std::make_shared<imstk::VisualObject>(planeName);
    object->setVisualGeometry(surfaceMesh); // change to any mesh created above
    scene->addSceneObject(object);
}

///
/// \brief Create a plane overlaid with a texture give translation, scale and rotation
///
void createPlaneTargetWithTexture(
    const std::shared_ptr<imstk::Scene>& scene,
    const double s,
    const Eigen::Translation3d& t,
    const Eigen::Quaterniond& r,
    const std::string& texFileName,
    const std::string& planeName)
{

    // Read surface mesh
    auto objMesh = imstk::MeshReader::read("Resources/plane3.obj");
    auto surfaceMesh = std::dynamic_pointer_cast<imstk::SurfaceMesh>(objMesh);
    surfaceMesh->addTexture(texFileName);

    // position the plane
    surfaceMesh->scale(s);
    surfaceMesh->translate(t.x(), t.y(), t.z());
    surfaceMesh->rotate(r);

    // Create object and add to scene
    auto object = std::make_shared<imstk::VisualObject>(planeName);
    object->setVisualGeometry(surfaceMesh); // change to any mesh created above
    scene->addSceneObject(object);
}


///
/// \brief Create the target blocks
///
void createTargetsScenario1(std::shared_ptr<imstk::Scene>& scene)
{
    // some constants
    const float X = 8;
    const float Y = 6;
    const float Z = 6;
    const float pY = 0.25;
    const float pZ = 0.25;
    const double radius = 4.5;//3.0;
    const double scaling = 0.25;// 0.15;
    const double planeWidth = 10;

    //imstk::Color meshColor(0.25, 0.25, 0.25, 1.0);

    //auto blockRenderDetail = std::make_shared<imstk::RenderDetail>();//IMSTK_RENDER_NORMALS
    //blockRenderDetail->setAmbientColor(meshColor);
    //blockRenderDetail->setDiffuseColor(meshColor);
    //blockRenderDetail->setSpecularColor(meshColor);
    //blockRenderDetail->setShininess(100.0);

    for (int i = 0; i < 6; i++)
    {
        // transformations
        Eigen::UniformScaling<double> s(scaling);
        Eigen::Translation3d t1(0, 0, -radius);
        Eigen::Translation3d t2(0, 0, -radius + 0.01);
        Eigen::Quaterniond q(cos(i*22.0 / 42), 0, sin(i*22.0 / 42), 0);
        q.normalize();

        // BLOCKS
        // surface mesh
        std::vector<imstk::Vec3d> blockPts = { imstk::Vec3d(X / 2, 0, -Z / 2), imstk::Vec3d(X / 2, 0, Z / 2),
            imstk::Vec3d(-X / 2, 0, Z / 2), imstk::Vec3d(-X / 2, 0, -Z / 2),
            imstk::Vec3d(-X / 2, Y, -Z / 2), imstk::Vec3d(X / 2, Y, -Z / 2),
            imstk::Vec3d(-X / 2, Y, Z*(pZ - 0.5)), imstk::Vec3d(X / 2, Y, Z*(pZ - 0.5)),
            imstk::Vec3d(-X / 2, Y*pY, Z / 2), imstk::Vec3d(X / 2, Y*pY, Z / 2) };

        for (int j = 0; j < 10; j++)
        {
            blockPts[j] *= scaling;
        }

        Vec3d texturePlaneNormal = (blockPts[7] - blockPts[6]).cross(blockPts[8] - blockPts[6]);
        texturePlaneNormal.normalize();

        /*std::cout << blockPts[6].x() << ", " << blockPts[6].y() << ", " << blockPts[6].z() << std::endl;
        std::cout << blockPts[7].x() << ", " << blockPts[7].y() << ", " << blockPts[7].z() << std::endl;
        std::cout << blockPts[8].x() << ", " << blockPts[8].y() << ", " << blockPts[8].z() << std::endl;
        std::cout << blockPts[9].x() << ", " << blockPts[9].y() << ", " << blockPts[9].z() << std::endl;*/

        std::vector<std::array<size_t, 3>> blockTriangles = { { { 0, 1, 2 } }, { { 0, 2, 3 } },
        { { 0, 3, 4 } }, { { 5, 0, 4 } },
        { { 5, 4, 6 } }, { { 7, 5, 6 } },
        { { 6, 8, 9 } }, { { 6, 9, 7 } },
        { { 2, 1, 9 } }, { { 8, 2, 9 } },
        { { 3, 6, 4 } }, { { 3, 8, 6 } },
        { { 3, 2, 8 } }, { { 5, 7, 0 } },
        { { 7, 9, 0 } }, { { 9, 1, 0 } } };

        // scale, translate, rotate (fix in architecture)
        /*for (int j = 0; j < 10; j++)
        {
        blockPts[j] *= scaling;
        blockPts[j] += imstk::Vec3d(0, 0, -radius);
        blockPts[j] = q*blockPts[j];
        }*/

        imstk::Vec3d pts[4];
        for (int k = 0; k < 4; k++)
        {
        pts[k] = blockPts[6 + k] * scaling;
        pts[k] += imstk::Vec3d(0, 0, -radius);
        pts[k] = q*pts[k];
        }


        auto blockMesh = std::make_shared<imstk::SurfaceMesh>();
        blockMesh->initialize(blockPts, blockTriangles, true);
        //blockMesh->scale(scaling);
        blockMesh->translate(Vec3d(0, 0, -radius));
        blockMesh->rotate(q);

        // add object to the scene
        auto blockObject = std::make_shared<imstk::VisualObject>("Target " + std::to_string(i));
        blockObject->setVisualGeometry(blockMesh);
        scene->addSceneObject(blockObject);

        //------------------------------------------------------

        std::string planeName("Plane " + std::to_string(i));
        std::string textureName("Resources/target.png");

        // Move the texture plane by delta to avoid coinciding with the plane of the block
        const double delta = -0.001;
        createPlaneTargetWithTexture(scene, 1, Eigen::Translation3d(texturePlaneNormal.x()*delta, texturePlaneNormal.y()*delta, texturePlaneNormal.z()*delta - radius), q, textureName, planeName);
    }
}


///
/// \brief Create the target blocks
///
void createTargetsScenario2(std::shared_ptr<imstk::Scene>& scene)
{
    // some constants
    const float X = 8;
    const float Y = 6;
    const float Z = 6;
    const float pY = 0.25;
    const float pZ = 0.25;
    const double radius = 3.0;
    const double scaling = 0.25;// 0.15;
    const double planeWidth = 10;

    //imstk::Color meshColor(0.25, 0.25, 0.25, 1.0);

    //auto blockRenderDetail = std::make_shared<imstk::RenderDetail>();//IMSTK_RENDER_NORMALS
    //blockRenderDetail->setAmbientColor(meshColor);
    //blockRenderDetail->setDiffuseColor(meshColor);
    //blockRenderDetail->setSpecularColor(meshColor);
    //blockRenderDetail->setShininess(100.0);

    int i = 0;
    // transformations
    Eigen::UniformScaling<double> s(scaling);
    Eigen::Translation3d t1(0, 0, -radius);
    Eigen::Translation3d t2(0, 0, -radius + 0.01);
    Eigen::Quaterniond q(cos(i*22.0 / 42), 0, sin(i*22.0 / 42), 0);
    q.normalize();

    // BLOCKS
    // surface mesh
    std::vector<imstk::Vec3d> blockPts = { imstk::Vec3d(X / 2, 0, -Z / 2), imstk::Vec3d(X / 2, 0, Z / 2),
        imstk::Vec3d(-X / 2, 0, Z / 2), imstk::Vec3d(-X / 2, 0, -Z / 2),
        imstk::Vec3d(-X / 2, Y, -Z / 2), imstk::Vec3d(X / 2, Y, -Z / 2),
        imstk::Vec3d(-X / 2, Y, Z*(pZ - 0.5)), imstk::Vec3d(X / 2, Y, Z*(pZ - 0.5)),
        imstk::Vec3d(-X / 2, Y*pY, Z / 2), imstk::Vec3d(X / 2, Y*pY, Z / 2) };

    for (int j = 0; j < 10; j++)
    {
        blockPts[j] *= scaling;
    }
    Vec3d texturePlaneNormal = (blockPts[7] - blockPts[6]).cross(blockPts[8] - blockPts[6]);
    texturePlaneNormal.normalize();

    /*std::cout << blockPts[6].x() << ", " << blockPts[6].y() << ", " << blockPts[6].z() << std::endl;
    std::cout << blockPts[7].x() << ", " << blockPts[7].y() << ", " << blockPts[7].z() << std::endl;
    std::cout << blockPts[8].x() << ", " << blockPts[8].y() << ", " << blockPts[8].z() << std::endl;
    std::cout << blockPts[9].x() << ", " << blockPts[9].y() << ", " << blockPts[9].z() << std::endl;*/

    std::vector<std::array<size_t, 3>> blockTriangles = { { { 0, 1, 2 } }, { { 0, 2, 3 } },
    { { 0, 3, 4 } }, { { 5, 0, 4 } },
    { { 5, 4, 6 } }, { { 7, 5, 6 } },
    { { 6, 8, 9 } }, { { 6, 9, 7 } },
    { { 2, 1, 9 } }, { { 8, 2, 9 } },
    { { 3, 6, 4 } }, { { 3, 8, 6 } },
    { { 3, 2, 8 } }, { { 5, 7, 0 } },
    { { 7, 9, 0 } }, { { 9, 1, 0 } } };

    // scale, translate, rotate (fix in architecture)
    /*for (int j = 0; j < 10; j++)
    {
    blockPts[j] *= scaling;
    blockPts[j] += imstk::Vec3d(0, 0, -radius);
    blockPts[j] = q*blockPts[j];
    }*/

    imstk::Vec3d pts[4];
    for (int k = 0; k < 4; k++)
    {
        pts[k] = blockPts[6 + k] * scaling;
        pts[k] += imstk::Vec3d(0, 0, -radius);
        pts[k] = q*pts[k];
    }


    auto blockMesh = std::make_shared<imstk::SurfaceMesh>();
    blockMesh->initialize(blockPts, blockTriangles, true);
    //blockMesh->scale(scaling);
    //blockMesh->translate(Vec3d(0, 0, -radius));
    //blockMesh->rotate(q);

    // add object to the scene
    auto blockObject = std::make_shared<imstk::VisualObject>("Target " + std::to_string(i));
    blockObject->setVisualGeometry(blockMesh);
    scene->addSceneObject(blockObject);

    //------------------------------------------------------

    std::string planeName("Plane " + std::to_string(i));
    std::string textureName("Resources/circle.png");

    const double delta = -0.001;
    //createPlaneTargetWithTexture(scene, 1, Eigen::Translation3d(texturePlaneNormal.x()*delta, texturePlaneNormal.y()*delta, texturePlaneNormal.z()*delta - radius), q, textureName, planeName);
    createPlaneTargetWithTexture(scene, 1, Eigen::Translation3d(texturePlaneNormal.x()*delta, texturePlaneNormal.y()*delta, texturePlaneNormal.z()*delta), q, textureName, planeName);

}


///
/// \brief Create the target blocks
///
void createTargetsScenario3(std::shared_ptr<imstk::Scene>& scene)
{
    // some constants
    const float X = 8;
    const float Y = 6;
    const float Z = 6;
    const float pY = 0.25;
    const float pZ = 0.25;
    const double radius = 3.0;
    const double scaling = 0.25;// 0.15;
    const double planeWidth = 10;

    //imstk::Color meshColor(0.25, 0.25, 0.25, 1.0);

    //auto blockRenderDetail = std::make_shared<imstk::RenderDetail>();//IMSTK_RENDER_NORMALS
    //blockRenderDetail->setAmbientColor(meshColor);
    //blockRenderDetail->setDiffuseColor(meshColor);
    //blockRenderDetail->setSpecularColor(meshColor);
    //blockRenderDetail->setShininess(100.0);

    int i = 0;
    // transformations
    Eigen::UniformScaling<double> s(scaling);
    Eigen::Translation3d t1(0, 0, -radius);
    Eigen::Translation3d t2(0, 0, -radius + 0.01);
    Eigen::Quaterniond q(cos(i*22.0 / 42), 0, sin(i*22.0 / 42), 0);
    q.normalize();

    // BLOCKS
    // surface mesh
    std::vector<imstk::Vec3d> blockPts = { imstk::Vec3d(X / 2, 0, -Z / 2), imstk::Vec3d(X / 2, 0, Z / 2),
        imstk::Vec3d(-X / 2, 0, Z / 2), imstk::Vec3d(-X / 2, 0, -Z / 2),
        imstk::Vec3d(-X / 2, Y, -Z / 2), imstk::Vec3d(X / 2, Y, -Z / 2),
        imstk::Vec3d(-X / 2, Y, Z*(pZ - 0.5)), imstk::Vec3d(X / 2, Y, Z*(pZ - 0.5)),
        imstk::Vec3d(-X / 2, Y*pY, Z / 2), imstk::Vec3d(X / 2, Y*pY, Z / 2) };

    for (int j = 0; j < 10; j++)
    {
        blockPts[j] *= scaling;
    }
    Vec3d texturePlaneNormal = (blockPts[7] - blockPts[6]).cross(blockPts[8] - blockPts[6]);
    texturePlaneNormal.normalize();

    /*std::cout << blockPts[6].x() << ", " << blockPts[6].y() << ", " << blockPts[6].z() << std::endl;
    std::cout << blockPts[7].x() << ", " << blockPts[7].y() << ", " << blockPts[7].z() << std::endl;
    std::cout << blockPts[8].x() << ", " << blockPts[8].y() << ", " << blockPts[8].z() << std::endl;
    std::cout << blockPts[9].x() << ", " << blockPts[9].y() << ", " << blockPts[9].z() << std::endl;*/

    std::vector<std::array<size_t, 3>> blockTriangles = { { { 0, 1, 2 } }, { { 0, 2, 3 } },
    { { 0, 3, 4 } }, { { 5, 0, 4 } },
    { { 5, 4, 6 } }, { { 7, 5, 6 } },
    { { 6, 8, 9 } }, { { 6, 9, 7 } },
    { { 2, 1, 9 } }, { { 8, 2, 9 } },
    { { 3, 6, 4 } }, { { 3, 8, 6 } },
    { { 3, 2, 8 } }, { { 5, 7, 0 } },
    { { 7, 9, 0 } }, { { 9, 1, 0 } } };

    // scale, translate, rotate (fix in architecture)
    /*for (int j = 0; j < 10; j++)
    {
    blockPts[j] *= scaling;
    blockPts[j] += imstk::Vec3d(0, 0, -radius);
    blockPts[j] = q*blockPts[j];
    }*/

    imstk::Vec3d pts[4];
    for (int k = 0; k < 4; k++)
    {
        pts[k] = blockPts[6 + k] * scaling;
        pts[k] += imstk::Vec3d(0, 0, -radius);
        pts[k] = q*pts[k];
    }


    auto blockMesh = std::make_shared<imstk::SurfaceMesh>();
    blockMesh->initialize(blockPts, blockTriangles, true);
    //blockMesh->scale(scaling);
    //blockMesh->translate(Vec3d(0, 0, -radius));
    //blockMesh->rotate(q);

    // add object to the scene
    auto blockObject = std::make_shared<imstk::VisualObject>("Target " + std::to_string(i));
    blockObject->setVisualGeometry(blockMesh);
    scene->addSceneObject(blockObject);

    std::string planeName("Plane " + std::to_string(i));
    std::string textureName("Resources/point.png");

    const double delta = -0.001;
    createPlaneTargetWithTexture(scene, 1, Eigen::Translation3d(texturePlaneNormal.x()*delta, texturePlaneNormal.y()*delta, texturePlaneNormal.z()*delta), q, textureName, planeName);

}