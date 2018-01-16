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

#include "imstkPointSet.h"

#include <limits>
#include <map>

#include <Eigen/Dense>

#define QUICKHULL_IMPLEMENTATION
#include "quickhull.h"

namespace imstk
{
void
PointSet::initialize(const StdVectorOfVec3d& vertices)
{
    this->setInitialVertexPositions(vertices);
    this->setVertexPositions(vertices);
}

void
PointSet::clear()
{
    m_initialVertexPositions.clear();
    m_vertexPositions.clear();
    m_vertexPositionsPostTransform.clear();
}

void
PointSet::print() const
{
    Geometry::print();
    LOG(INFO) << "Number of vertices: " << this->getNumVertices();
    LOG(INFO) << "Vertex positions:";
    for (auto &verts : m_vertexPositions)
    {
        LOG(INFO) << verts.x() << ", " << verts.y() << ", " << verts.z();
    }
}

void
PointSet::computeBoundingBox(Vec3d& min, Vec3d& max, const double percent) const
{
    min = Vec3d(MAX_D, MAX_D, MAX_D);
    max = Vec3d(-MAX_D, -MAX_D, -MAX_D);

    for (auto& pos : m_vertexPositions)
    {
        for (int i = 0; i < 3; ++i)
        {
            min[i] = std::min(min[i], pos[i]);
            max[i] = std::max(max[i], pos[i]);
        }
    }

    if (percent == 0.0)
    {
        return;
    }
    else
    {
        Vec3d range = max - min;
        min = min - range*(percent / 100);
        max = max + range*(percent / 100);
    }
}

void
PointSet::setInitialVertexPositions(const StdVectorOfVec3d& vertices)
{
    if (m_originalNumVertices == 0)
    {
        m_initialVertexPositions = vertices;
        m_originalNumVertices = vertices.size();
        m_maxNumVertices = (size_t)(m_originalNumVertices * m_loadFactor);
        m_vertexPositions.reserve(m_maxNumVertices);
    }
    else
    {
        LOG(WARNING) << "Already set initial vertices";
    }
}

const StdVectorOfVec3d&
PointSet::getInitialVertexPositions() const
{
    return m_initialVertexPositions;
}

const Vec3d&
PointSet::getInitialVertexPosition(const size_t& vertNum) const
{
    return m_initialVertexPositions.at(vertNum);
}

void
PointSet::setVertexPositions(const StdVectorOfVec3d& vertices)
{
    if (vertices.size() <= m_maxNumVertices)
    {
        m_vertexPositions = vertices;
        m_dataModified = true;
        m_transformApplied = false;
    }
    else
    {
        LOG(WARNING) << "Vertices not set, exceeded maximum number of vertices";
    }
}

const StdVectorOfVec3d&
PointSet::getVertexPositions(DataType type /* = DataType::PostTransform */)
{
    if (type == DataType::PostTransform)
    {
        this->updatePostTransformData();
        return m_vertexPositionsPostTransform;
    }
    return m_vertexPositions;
}

void
PointSet::setVertexPosition(const size_t& vertNum, const Vec3d& pos)
{
    m_vertexPositions.at(vertNum) = pos;
    m_dataModified = true;
    m_transformApplied = false;
}

const Vec3d&
PointSet::getVertexPosition(const size_t& vertNum, DataType type)
{
    return this->getVertexPositions(type).at(vertNum);
}

void
PointSet::setVertexDisplacements(const StdVectorOfVec3d& diff)
{
    assert(diff.size() == m_vertexPositions.size());
    for (size_t i = 0; i < m_vertexPositions.size(); ++i)
    {
        m_vertexPositions[i] = m_initialVertexPositions[i] + diff[i];
    }
    m_dataModified = true;
    m_transformApplied = false;
}

void
PointSet::setVertexDisplacements(const Vectord& u)
{
    assert(u.size() == 3 * m_vertexPositions.size());
    size_t dofId = 0;
    for (size_t i = 0; i < m_vertexPositions.size(); ++i)
    {
        m_vertexPositions[i] = m_initialVertexPositions[i] + Vec3d(u(dofId), u(dofId + 1), u(dofId + 2));
        dofId += 3;
    }
    m_dataModified = true;
    m_transformApplied = false;
}

void
PointSet::translateVertices(const Vec3d& t)
{
    for (size_t i = 0; i < m_vertexPositions.size(); ++i)
    {
        m_vertexPositions[i] += t;
    }
    m_dataModified = true;
    m_transformApplied = false;
}

void
PointSet::setPointDataMap(const std::map<std::string, StdVectorOfVectorf>& pointData)
{
    m_pointDataMap = pointData;
}

const std::map<std::string, StdVectorOfVectorf>&
PointSet::getPointDataMap() const
{
    return m_pointDataMap;
}

void
PointSet::setPointDataArray(const std::string& arrayName, const StdVectorOfVectorf& arrayData)
{
    if ( arrayData.size() != this->getNumVertices())
    {
        LOG(WARNING) << "Specified array should have " << this->getNumVertices()
                     << " tuples, has " << arrayData.size();
        return;
    }
    m_pointDataMap[arrayName] = arrayData;
}

const StdVectorOfVectorf*
PointSet::getPointDataArray(const std::string& arrayName) const
{
    auto it = m_pointDataMap.find(arrayName);
    if (it == m_pointDataMap.end())
    {
        LOG(WARNING) << "No array with such name holds any point data.";
        return nullptr;
    }
    return &(it->second);
}

size_t
PointSet::getNumVertices() const
{
    return m_vertexPositions.size();
}


void
PointSet::applyTranslation(const Vec3d t)
{
    for (size_t i = 0; i < m_vertexPositions.size(); ++i)
    {
        m_vertexPositions[i] += t;
        m_initialVertexPositions[i] += t;
    }
    m_dataModified = true;
    m_transformApplied = false;
}

void
PointSet::applyRotation(const Mat3d r)
{
    for (size_t i = 0; i < m_vertexPositions.size(); ++i)
    {
        m_vertexPositions[i] = r * m_vertexPositions[i];
        m_initialVertexPositions[i] = r * m_initialVertexPositions[i];
    }
    m_dataModified = true;
    m_transformApplied = false;
}

void
PointSet::applyScaling(const double s)
{
    for (size_t i = 0; i < m_vertexPositions.size(); ++i)
    {
        m_vertexPositions[i] = s * m_vertexPositions[i];
        m_initialVertexPositions[i] = s * m_initialVertexPositions[i];
    }
    m_dataModified = true;
    m_transformApplied = false;
}

void
PointSet::updatePostTransformData()
{
    if (m_transformApplied)
    {
        return;
    }

    if (m_vertexPositionsPostTransform.size() != m_vertexPositions.size())
    {
        m_vertexPositionsPostTransform.clear();
        m_vertexPositionsPostTransform.resize(m_vertexPositions.size());
    }
    for (size_t i = 0; i < m_vertexPositions.size(); ++i)
    {
        // NOTE: Right now scaling is appended on top of the rigid transform
        // for scaling around the mesh center, and not concatenated within
        // the transform, for ease of use.
        m_vertexPositionsPostTransform[i] = m_transform * (m_vertexPositions[i]* m_scaling);
    }
    m_transformApplied = true;
}

void
PointSet::setLoadFactor(double loadFactor)
{
    m_loadFactor = loadFactor;
    m_maxNumVertices = (size_t)(m_originalNumVertices * m_loadFactor);
    m_vertexPositions.reserve(m_maxNumVertices);
}

double
PointSet::getLoadFactor()
{
    return m_loadFactor;
}

size_t
PointSet::getMaxNumVertices()
{
    return m_maxNumVertices;
}

Graph
PointSet::getMeshGraph()
{
    LOG(WARNING) << "The graph of a point set has no edges";

    return Graph(this->getNumVertices());
}

convexHull
PointSet::computeConvexHullQuickHull() const
{
    const auto totalVert = m_vertexPositions.size();
    qh_vertex_t* vertices = new qh_vertex_t[totalVert];
    for (int p = 0; p < totalVert; ++p)
    {
        /*vertices[p].x = m_vertexPositions[p][0];
        vertices[p].y = m_vertexPositions[p][1];
        vertices[p].z = m_vertexPositions[p][2];*/

        for (int i = 0; i < 3; ++i)
        {
            vertices[p].v[i] = m_vertexPositions[p][i];
        }
    }
    qh_mesh_t mesh = qh_quickhull3d(vertices, totalVert);

    convexHull hull;
    // add vertices
    for (unsigned int i = 0; i < mesh.nvertices; ++i)
    {
        hull.m_vertices.push_back(Vec3d(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z));
    }
    // add faces
    for (unsigned int i = 0; i < mesh.nindices; i += 3)
    {
        hull.m_faces.push_back(convexHull::hullTriFace { {mesh.indices[i], mesh.indices[i+1], mesh.indices[i+2]} });
    }
    qh_free_mesh(mesh);

    return std::move(hull);
}

convexHull
PointSet::computeConvexHull() const
{
    // Start of incremental convex hull algorithm
    std::vector<hullFace> faces;
    const size_t numVerts = m_vertexPositions.size();
    std::vector<std::vector<hullEdge>> E;
    E.resize(numVerts);
    for (size_t i = 0; i < numVerts; i++)
    {
        for (size_t j = 0; j < numVerts; j++)
        {
            E[i].push_back(hullEdge(-1, -1));
        }
    }

    ///
    /// \brief Compute the half plane {x : c^T norm < disc}
    /// defined by the three points A[i], A[j], A[k] where
    /// A[inside_i] is considered to be on the 'interior' side of the face
    ///
    auto makeHullFace = [&](const size_t i, const size_t j, const size_t k, const size_t inside_i) -> hullFace
                        {
                            E[i][j].insert(k);
                            E[i][k].insert(j);
                            E[j][k].insert(i);

                            hullFace f(i, j, k);
                            f.m_normal = (m_vertexPositions[j] - m_vertexPositions[i]).cross(m_vertexPositions[k] - m_vertexPositions[i]);
                            f.disc = f.m_normal.dot(m_vertexPositions[i]);
                            if (f.m_normal.dot(m_vertexPositions[inside_i]) > f.disc)
                            {
                                f.m_normal = -f.m_normal;
                                f.disc = -f.disc;
                            }
                            return f;
                        };

    // Initially construct the hull as containing only the first four points
    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = i + 1; j < 4; j++)
        {
            for (size_t k = j + 1; k < 4; k++)
            {
                faces.push_back(makeHullFace(i, j, k, 6 - i - j - k));
            }
        }
    }

    // Now add a point into the hull one at a time
    for (size_t i = 4; i < numVerts; i++)
    {
        hullFace f;
        // Find and delete all faces with their outside 'illuminated' by this point
        for (size_t j = 0; j < faces.size(); j++)
        {
            f = faces[j];
            if (f.m_normal.dot(m_vertexPositions[i]) > f.disc)
            {
                E[f.m_faceCorners[0]][f.m_faceCorners[1]].erase(f.m_faceCorners[2]);
                E[f.m_faceCorners[0]][f.m_faceCorners[2]].erase(f.m_faceCorners[1]);
                E[f.m_faceCorners[1]][f.m_faceCorners[2]].erase(f.m_faceCorners[0]);
                faces[j--] = faces.back();
                faces.resize(faces.size() - 1);
            }
        }
        // Now for any edge still in the hull that is only part of one face
        // add another face containing the new point and that edge to the hull
        size_t nfaces = faces.size();
        for (size_t j = 0; j < nfaces; j++)
        {
            f = faces[j];
            for (int a = 0; a < 3; a++)
            {
                for (int b = a + 1; b < 3; b++)
                {
                    if (E[f.m_faceCorners[a]][f.m_faceCorners[b]].size() == 2)
                    {
                        continue;
                    }
                    faces.push_back(makeHullFace(f.m_faceCorners[a], f.m_faceCorners[b], i, f.m_faceCorners[3 - a - b]));
                }
            }
        }
    }

    // mark the subset of vertices that are part of the convex hull
    std::vector<bool> hullVerts(numVerts, false);
    for (auto currFace : faces)
    {
        hullVerts[currFace.m_faceCorners[0]] = true;
        hullVerts[currFace.m_faceCorners[1]] = true;
        hullVerts[currFace.m_faceCorners[2]] = true;
    }

    size_t hullVertId = 0;
    std::map<size_t, size_t> vertIdMap;
    for (auto i = 0; i < hullVerts.size(); i++)
    {
        if (hullVerts[i])
        {
            vertIdMap[i] = hullVertId;
            hullVertId++;
        }
    }

    convexHull convHull;
    // Create hull vertices
    size_t vertId = 0;
    for (auto currHullVertex : hullVerts)
    {
        if (currHullVertex)
        {
            convHull.m_vertices.push_back(m_vertexPositions[vertId]);
        }
        vertId++;
    }

    // Create hull faces
    for (auto currFace : faces)
    {
        auto a = vertIdMap.find(currFace.m_faceCorners[0]);
        auto b = vertIdMap.find(currFace.m_faceCorners[1]);
        auto c = vertIdMap.find(currFace.m_faceCorners[2]);

        /*if (a != vertIdMap.end() && b != vertIdMap.end() && c != vertIdMap.end())
        {
            LOG(WARNING) << "Could not find the face corners from the map!";
        }*/

        convHull.m_faces.push_back(std::array < size_t, 3 > {{a->second, b->second, c->second}});
    }

    return std::move(convHull);
}

OBB
PointSet::computeOBB() const
{
    return this->evaluateOBB(m_vertexPositions);
}

OBB
PointSet::evaluateOBB(const StdVectorOfVec3d& points)
{
    const auto numPoints = points.size();

    // compute mean
    Vec3d mean(0., 0., 0.);
    double min[3] = { MAX_D, MAX_D, MAX_D };
    double max[3] = { MIN_D, MIN_D,MIN_D };
    for (auto point : points)
    {
        for (int i = 0; i < 3; ++i)
        {
            min[i] = std::min(point[i], min[i]);
            max[i] = std::max(point[i], max[i]);
        }
        mean += point;
    }
    mean /= numPoints;
    Vec3d mid = Vec3d(min[0], min[1], min[2]) + (Vec3d(max[0], max[1], max[2]) - Vec3d(min[0], min[1], min[2]))*0.5;


    // Build the covariance matrix
    double Exx = 0., Eyy = 0., Ezz = 0., Exy = 0., Exz = 0., Eyz = 0.;
    for (auto point : points)
    {
        Exx += point.x()*point.x();
        Eyy += point.y()*point.y();
        Ezz += point.z()*point.z();
        Exy += point.x()*point.y();
        Exz += point.x()*point.z();
        Eyz += point.y()*point.z();
    }
    Mat3d covarianceMat;
    covarianceMat(0, 0) = Exx / numPoints + mean.x()*mean.x();
    covarianceMat(1, 1) = Eyy / numPoints + mean.y()*mean.y();
    covarianceMat(2, 2) = Ezz / numPoints + mean.z()*mean.z();
    covarianceMat(0, 1) = covarianceMat(1, 0) = Exy / numPoints + mean.x()*mean.y();
    covarianceMat(0, 2) = covarianceMat(2, 0) = Exz / numPoints + mean.x()*mean.z();
    covarianceMat(1, 2) = covarianceMat(2, 1) = Eyz / numPoints + mean.y()*mean.z();

    Eigen::SelfAdjointEigenSolver<Mat3d> eigensolver;
    eigensolver.compute(covarianceMat);
    auto ev = eigensolver.eigenvectors();

    OBB Obb;
    Obb.m_center = mid; // assign center

    // Find the OBB dimensions
    Vec3d dim[3];
    for (int i = 0; i < 3; i++)
    {
        Obb.m_axis[i] = ev.col(i); // assign axis
        Obb.m_axis[i].normalize();

        // find range about the axis
        double min = MAX_D, max = MIN_D;
        for (auto point : points)
        {
            Vec3d s = point - mid;
            auto dist = (s*(1. - s.dot(Obb.m_axis[i]) / s.norm())).norm();

            if (dist < min)
            {
                min = dist;
            }

            if (dist > max)
            {
                max = dist;
            }
        }
        Obb.m_halfLengths[i] = (max - min) / 2.; // assign half length
    }
    return Obb;
}
} // imstk
