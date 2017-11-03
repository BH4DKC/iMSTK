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

#ifndef imstkConvexHull_h
#define imstkConvexHull_h

#include "imstkMath.h"

#include <array>

namespace imstk
{
///
/// \brief Face of the convex hull
///
struct hullFace
{
    hullFace() = default;
    hullFace(const size_t i, const size_t j, const size_t k)
    {
        m_faceCorners[0] = i;
        m_faceCorners[1] = j;
        m_faceCorners[2] = k;
    }
    Vec3d m_normal;
    double disc;
    size_t m_faceCorners[3];
};

///
/// \brief E[i][j] indicates which (up to two) other points combine with the edge i and
/// j to make a face in the hull.  Only defined when i < j.
///
struct hullEdge
{
    hullEdge() = default;
    hullEdge(const size_t i, const size_t j) : a(i), b(i){}

    void insert(const size_t x)
    {
        (a == -1 ? a : b) = x;
    }

    bool contains(const size_t x)
    {
        return a == x || b == x;
    }

    void erase(const size_t x)
    {
        (a == x ? a : b) = -1;
    }

    size_t size()
    {
        return (a != -1) + (b != -1);
    }

    size_t a, b; // vertex IDs of the edge
};

///
/// \brief Convex hull data structure
///
struct convexHull
{
    using hullTriFace = std::array<size_t, 3>;

    StdVectorOfVec3d m_vertices;
    std::vector<hullTriFace> m_faces;

    void print() const
    {
        std::cout << "Convex Hull:\n";
        std::cout << "\tPoints:\n";
        for (const auto& point : m_vertices)
        {
            std::cout << "\t" << point.adjoint() << "\n";
        }
        std::cout << "\tFaces:\n";
        for (const auto& face : m_faces)
        {
            std::cout << "\t" << face[0] << ", " << face[1] << ", " << face[2] << "\n";
        }
        std::cout << std::endl;
    }
};
}

#endif //imstkConvexHull_h
