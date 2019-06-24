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

#pragma once

#include <memory>

#include "imstkVTKRenderDelegate.h"
#include "imstkCylinder.h"

#include "vtkTransformPolyDataFilter.h"

#include "vtkCylinderSource.h"
namespace imstk
{
///
/// \class CylinderRenderDelegate
///
/// \brief Cylinder render delegate
///
class VTKCylinderRenderDelegate : public VTKRenderDelegate
{
public:
    ///
    /// \brief Constructor
    ///
    VTKCylinderRenderDelegate(std::shared_ptr<VisualModel> visualModel);

    ///
    /// \brief Destructor
    ///
    ~VTKCylinderRenderDelegate() = default;

    ///
    /// \brief Update cylinder source based on the cylinder geometry
    ///
    void updateDataSource() override;

protected:
    vtkSmartPointer<vtkCylinderSource> cylinderSource;

    vtkSmartPointer<vtkTransformPolyDataFilter> m_transformFilter; ///> Source
};
} // imstk
