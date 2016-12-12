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

#include "imstkGeometry.h"

namespace imstk
{

void
Geometry::print() const
{
    LOG(INFO) << this->getTypeName();
    LOG(INFO) << "Position: " << "(" << m_position.x() << ", " << m_position.y() << ", " << m_position.z() << ")";
    LOG(INFO) << "Orientation:\n" << m_orientation.toRotationMatrix();
    LOG(INFO) << "Scaling: " << m_scaling;
    LOG(INFO) << "Color: " << m_color;
}

void
Geometry::translate(const Vec3d& t)
{
    m_position += t;

	m_transform = Eigen::Translation3d(t)*m_transform;
	m_configurationModified = true;
}

void
Geometry::translate(const double& x,
                    const double& y,
                    const double& z)
{
    this->translate(Vec3d(x, y, z));
}

void
Geometry::rotate(const Quatd& r)
{
    m_orientation = r * m_orientation;

	m_transform = r*m_transform;
	m_configurationModified = true;
}

void
Geometry::rotate(const Mat3d& r)
{
    this->rotate(Quatd(r));
}

void
Geometry::rotate(const Vec3d& axis, const double& angle)
{
    this->rotate(Quatd(Rotd(angle, axis)));
}

void
Geometry::scale(const double& scaling)
{
    m_scaling *= scaling;
	
	m_transform = Eigen::Scaling(scaling)*m_transform;
	m_configurationModified = true;
}

void
Geometry::transform(const RigidTransform3d& transform)
{
    this->rotate(transform.rotation());
	this->translate(transform.translation());
}

bool
Geometry::isMesh() const
{
    return (this->m_type == Type::HexahedralMesh ||
            this->m_type == Type::SurfaceMesh ||
            this->m_type == Type::TetrahedralMesh ||
			this->m_type == Type::LineMesh
			) ? true : false;
}

const Vec3d&
Geometry::getPosition() const
{
    return m_position;
}

void
Geometry::setPosition(const Vec3d& position)
{
    m_position = position;

	m_transform.translation() = position;
	m_configurationModified = true;
}

void
Geometry::setPosition(const double& x,
                      const double& y,
                      const double& z)
{
    this->setPosition(Vec3d(x, y, z));
}

const Quatd&
Geometry::getOrientation() const
{
    return m_orientation;
}

void
Geometry::setOrientation(const Quatd& orientation)
{
    m_orientation = orientation;

	m_transform.linear() = Mat3d(orientation);

	// reapply scaling
	m_transform.scale(m_scaling);
	m_configurationModified = true;
}

void
Geometry::setOrientation(const Mat3d& orientation)
{
    this->setOrientation(Quatd(orientation));
}

void
Geometry::setOrientation(const Vec3d& axis, const double& angle)
{
    this->setOrientation(Quatd(Rotd(angle, axis)));
}

const double&
Geometry::getScaling() const
{
    return m_scaling;
}

void
Geometry::setScaling(const double& scaling)
{
    m_scaling = scaling;

	m_transform.scale(m_scaling);
	m_configurationModified = true;
}

const Geometry::Type&
Geometry::getType() const
{
    return m_type;
}

void
Geometry::resetConfiguration()
{
	m_transform.setIdentity();
	m_configurationModified = true;
}

void 
Geometry::setColorProperty(double r, double g, double b, double a)
{
    m_color.setValue(r, g, b, a);
}

const std::string
Geometry::getTypeName() const
{
    switch (m_type)
    {
    case Type::Cube:
        return "Cube";
    case Type::Plane:
        return "Plane";
    case Type::Sphere:
        return "Sphere";
    case Type::SurfaceMesh:
        return "Surface triangular mesh";
    case Type::TetrahedralMesh:
        return "Tetrahedral mesh";
    case Type::HexahedralMesh:
        return "Hexahedral Mesh";
    default:
        return "Mesh type not determined!";
    }
}

} // imstk