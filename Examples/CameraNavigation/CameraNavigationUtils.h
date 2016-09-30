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

const int overlaySize = 400;
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
void add2DTextureOverlay(vtkSmartPointer<vtkRenderer> rendererVtk, const char* fileName)
{
	// Read the image
	vtkNew<vtkPNGReader> reader;
	reader->SetFileName(fileName);
	reader->Update();

	int dim[3] = { overlaySize, overlaySize, 1 };

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
/// \brief Create a plane overlaid with a texture
///
void createPlaneTargetWithTexture(
	std::shared_ptr<imstk::Scene>& scene, 
	double s, 
	Eigen::Translation3d& t,
	Eigen::Quaterniond& r, 
	std::string& texFileName, 
	std::string& planeName)
{

	// Read surface mesh
	auto objMesh = imstk::MeshReader::read("Resources/plane.obj");
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