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

#ifndef imstkLineMesh_h
#define imstkLineMesh_h

#include <memory>

#include "imstkPointSet.h"


namespace imstk
{
///
/// \class LineMesh
///
/// \brief Base class for all volume mesh types
///
class LineMesh : public PointSet
{
public:

    ///
    /// \brief Constructor
    ///
    LineMesh() : PointSet(Geometry::Type::LineMesh) {}

    ///
    /// \brief Default destructor
    ///
    ~LineMesh() = default;

    ///
    /// \brief
    ///
    void clear() override;

    ///
    /// \brief
    ///
    void print() const override;

    ///
    /// \brief
    ///
    double getVolume() const override;

    ///
    /// \brief
    ///
    void setConnectivity(const std::vector<std::vector<int>>& lines);

    ///
    /// \brief
    ///
    size_t getNumLines();

    ///
    /// \brief
    ///
    std::vector<std::vector<int>> getLines() const;

    ///
    /// \brief
    ///
    std::vector<int> getLine(int index) const;

private:

    friend class VTKLineMeshRenderDelegate;

    std::vector<std::vector<int>> m_lines;  ///> line connectivity
};
} // imstk

#endif // ifndef imstkLineMesh_h