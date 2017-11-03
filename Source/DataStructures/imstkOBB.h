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

#ifndef imstkOBB_h
#define imstkOBB_h

#include "imstkMath.h"
#include <Eigen/Eigenvalues>

namespace imstk
{
///
/// \brief Oriented bounding box data structure
///
struct OBB
{
    Vec3d m_center;
    Vec3d m_halfLengths;
    Vec3d m_axis[3];

    void print() const
    {
        std::cout << "OBB:\n";
        std::cout << "\tCenter: " << m_center.adjoint() << "\n";
        std::cout << "\tHalf-lengths: " << m_halfLengths.adjoint() << "\n";
        std::cout << "\tAxis 1: " << m_axis[0].adjoint() << "\n";
        std::cout << "\tAxis 2: " << m_axis[1].adjoint() << "\n";
        std::cout << "\tAxis 3: " << m_axis[2].adjoint() << std::endl;
    }

    StdVectorOfVec3d getCorners()
    {
        StdVectorOfVec3d corners;
        const auto hl = m_halfLengths*1.05;

        corners.push_back(m_center - m_axis[0] * hl[0] - m_axis[1] * hl[1] - m_axis[2] * hl[2]);
        corners.push_back(m_center + m_axis[0] * hl[0] - m_axis[1] * hl[1] - m_axis[2] * hl[2]);
        corners.push_back(m_center + m_axis[0] * hl[0] - m_axis[1] * hl[1] + m_axis[2] * hl[2]);
        corners.push_back(m_center - m_axis[0] * hl[0] - m_axis[1] * hl[1] + m_axis[2] * hl[2]);
        
        corners.push_back(m_center - m_axis[0] * hl[0] + m_axis[1] * hl[1] - m_axis[2] * hl[2]);
        corners.push_back(m_center + m_axis[0] * hl[0] + m_axis[1] * hl[1] - m_axis[2] * hl[2]);
        corners.push_back(m_center + m_axis[0] * hl[0] + m_axis[1] * hl[1] + m_axis[2] * hl[2]);
        corners.push_back(m_center - m_axis[0] * hl[0] + m_axis[1] * hl[1] + m_axis[2] * hl[2]);

        return std::move(corners);        
    }
        
};
}

#endif //imstkOBB_h
