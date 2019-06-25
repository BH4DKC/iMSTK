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
#include "imstkPlane.h"

#include "vtkTransformPolyDataFilter.h"

namespace imstk
{
///
/// \class PlaneRenderDelegate
///
/// \brief
///
class VTKPlaneRenderDelegate : public VTKRenderDelegate
{
public:
    ///
    /// \brief Constructor
    ///
    VTKPlaneRenderDelegate(std::shared_ptr<VisualModel> visualModel);

    ///
    /// \brief Destructor
    ///
    ~VTKPlaneRenderDelegate() = default;

    ///
    /// \brief Update plane source based on the plane geometry
    ///
    void updateDataSource() override;

protected:

    vtkSmartPointer<vtkTransformPolyDataFilter> m_transformFilter;
};
} // imstk
