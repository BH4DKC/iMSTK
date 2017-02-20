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

#include "imstkRenderDelegate.h"

#include "g3log/g3log.hpp"

#include "imstkGeometry.h"
#include "imstkPlane.h"
#include "imstkSphere.h"
#include "imstkCube.h"
#include "imstkSurfaceMesh.h"
#include "imstkLineMesh.h"
#include "imstkTetrahedralMesh.h"
#include "imstkHexahedralMesh.h"
#include "imstkPlaneRenderDelegate.h"
#include "imstkSphereRenderDelegate.h"
#include "imstkCubeRenderDelegate.h"
#include "imstkSurfaceMeshRenderDelegate.h"
#include "imstkLineMeshRenderDelegate.h"
#include "imstkTetrahedralMeshRenderDelegate.h"

#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkTransform.h"
#include "vtkProperty.h"

namespace imstk
{

std::shared_ptr<RenderDelegate>
RenderDelegate::make_delegate(std::shared_ptr<Geometry>geom)
{
    switch (geom->getType())
    {
    case Geometry::Type::Plane:
    {
        auto plane = std::dynamic_pointer_cast<Plane>(geom);
        return std::make_shared<PlaneRenderDelegate>(plane);
    }
    case Geometry::Type::Sphere:
    {
        auto sphere = std::dynamic_pointer_cast<Sphere>(geom);
        return std::make_shared<SphereRenderDelegate>(sphere);
    }
    case Geometry::Type::Cube:
    {
        auto cube = std::dynamic_pointer_cast<Cube>(geom);
        return std::make_shared<CubeRenderDelegate>(cube);
    }
    case Geometry::Type::SurfaceMesh:
    {
        auto surface = std::dynamic_pointer_cast<SurfaceMesh>(geom);
        return std::make_shared<SurfaceMeshRenderDelegate>(surface);
    }
    case Geometry::Type::TetrahedralMesh:
    {
        auto mesh = std::dynamic_pointer_cast<TetrahedralMesh>(geom);
        return std::make_shared<TetrahedralMeshRenderDelegate>(mesh);
    }
	case Geometry::Type::LineMesh:
	{
		auto mesh = std::dynamic_pointer_cast<LineMesh>(geom);
		return std::make_shared<LineMeshRenderDelegate>(mesh);
	}
    case Geometry::Type::HexahedralMesh:
    {
        auto mesh = std::dynamic_pointer_cast<TetrahedralMesh>(geom);
        LOG(WARNING) << "RenderDelegate::make_delegate error: HexahedralMeshRenderDelegate not yet implemented";
        return nullptr;
    }
    default:
    {
        LOG(WARNING) << "RenderDelegate::make_delegate error: Geometry type incorrect.";
        return nullptr;
    }
    }
}

void
RenderDelegate::setActorMapper(vtkAlgorithmOutput *source)
{

    auto normalGen = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGen->SetInputConnection(source);
    normalGen->SplittingOff();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(normalGen->GetOutputPort());

    m_actor->SetMapper(mapper);
    this->setColorAndOpacity(); 
    this->setWireFrameMode();

}

vtkSmartPointer<vtkActor>
RenderDelegate::getVtkActor() const
{
    return m_actor;
}

void
RenderDelegate::update()
{
    // TODO : only when rigid transform applied
    this->updateActorTransform();
    this->updateVtkProperties();
}

void
RenderDelegate::setColorAndOpacity()
{
    imstk::Color geomColor = this->getGeometry()->getGeometryColor();
    this->m_actor->GetProperty()->SetColor(geomColor.r, geomColor.g, geomColor.b);
    this->m_actor->GetProperty()->SetOpacity(geomColor.a);
    this->m_actor->Modified();
}

void 
RenderDelegate::setWireFrameMode()
{
    if (this->getGeometry()->getWireFrameMode())
        this->m_actor->GetProperty()->SetRepresentationToWireframe();
    else
        this->m_actor->GetProperty()->SetRepresentationToSurface();
    this->m_actor->Modified();
}

void
RenderDelegate::setVtkTrasnformFromEigen(const AffineTransform3d& t)
{
	auto em = t.matrix();
	double m[16];

	// copy data
	m[0] = em(0, 0); 
	m[1] = em(0, 1);
	m[2] = em(0, 2);
	m[3] = em(0, 3);

	m[4] = em(1, 0);
	m[5] = em(1, 1);
	m[6] = em(1, 2);
	m[7] = em(1, 3);

	m[8] = em(2, 0);
	m[9] = em(2, 1);
	m[10] = em(2, 2);
	m[11] = em(2, 3);

	m[12] = em(3, 0);
	m[13] = em(3, 1);
	m[14] = em(3, 2);
	m[15] = em(3, 3);

	m_transform->SetMatrix(m);
}

void
RenderDelegate::updateActorTransform()
{
	if (this->getGeometry()->isConfigurationModified())
	{
		setVtkTrasnformFromEigen(this->getGeometry()->getEigenTransform());
		m_actor->SetUserTransform(m_transform);

		this->getGeometry()->setConfigurationModified(false);
	} 
}

void
RenderDelegate::updateVtkProperties()
{
    if (this->getGeometry()->isVtkPropertyModified())
    {
        this->setColorAndOpacity();
        this->setWireFrameMode();
        this->getGeometry()->seVtkPropertyModified(false);
    }
}

} // imstk