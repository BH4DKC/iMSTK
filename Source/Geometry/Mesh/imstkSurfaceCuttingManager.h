#pragma once

#include "imstkSurfaceMesh.h"
#include "imstkToolSurfaceInteractionPair.h"
#include <iostream>

namespace imstk
{

struct MeshSplitTriangle
{
    size_t triId;       // id split triangle in the global tris list
	std::array<size_t, 3> nodeId;
	size_t newNodeId;   // new node id
	Vec3f baryCoords;   // barycentric coords of the new node
    MeshSplitTriangle() {}
	MeshSplitTriangle(size_t tId, std::array<size_t, 3> vId) : triId(tId), nodeId(vId), newNodeId(0)
	{}
};

struct MeshBrokenEdge
{
    bool isConnected = true;
    int idNeiTriToBeCut;
    std::vector<size_t> idNeiTri;
	std::array<size_t, 2> nodeId; 
	std::array<size_t, 2> newNodeId; 
	float brokenCoord;	  //broken position = (1-brokenCoord)*v1 + brokenCoord*v2
    MeshBrokenEdge(){}
	MeshBrokenEdge(size_t id1, size_t id2, float broken = 0.5) :
		brokenCoord(broken)
	{
        nodeId = { id1, id2 };
        idNeiTri.resize(0);
	}
};

struct MeshTriangle //Carvable Triangle element of the Triangle Mesh
{
	std::array<size_t, 3> nodeId; // vertices id
	bool isBroken;        // if the triangle has been cut
	int numberCuts;		  // num of broken edges, supposed to be 0~2; 2 means full-cut triangle, 1 means partial-cut triangle
	int brokenType;		  // indicating where the cuts are: idx of the uncut edge for full-cut triangle; idx of the cut edge for partial-cut triangle
	std::vector<int> brokenEdgeId; //broken edge id in the m_brokenEdges
	bool doneHandlingCut; // prevent repeated handling cut
    bool canBeCut = true; // not cuttable
    bool tooSmallToCut;   // area smaller than the threshold
	MeshTriangle(std::array<size_t, 3> verts) : nodeId(verts), isBroken(false), tooSmallToCut(false), numberCuts(0), brokenType(-1)
	{
		doneHandlingCut = false;
        brokenEdgeId.resize(0);
	}

};


class SurfaceCuttingManager : public SurfaceMesh
{
public:
	friend class PbdModel;
	///
	/// \brief Constructor
	///
    SurfaceCuttingManager() : SurfaceMesh()
	{}
	  
	///
	/// \brief Default destructor
	///
	~SurfaceCuttingManager() = default;

	///
	/// \brief initialize the global triangles list
	/// Note : this also initialize the surfaceMesh m_triangleVertices data
    void initializeTriangles(const std::vector<TriangleArray>& triangles);



	/// DoCutting -- Remeshing based on the broken info from generateCut()
	/// Remeshing should happen in the folllowing manner:
	///		1. Traverse all m_triangles:
	///			Mark all broken triangles and their types
	///			Find all BrokenCoords and Add to the list of MeshBrokenEdges
	///		2. Traverse all m_brokenEdges:
	///			Create 2 new vertices on each edge
	///		3. Go through all m_triangles:
	///			Split every triangle that has 2 cuts into 3 subtriangles
	///			Split every triangle that has 1 cuts into 4 subtriangles
	///		4. Propagate the topology changes to PbdModel:
	///			Copy new triangles from m_trianlges to the m_trianglesVertices
	///			Delete broken triangles in the m_trianglesVertices
	///		TODO: For Multigrid mesh, use this m_triangles to store the list of all triangles among all levels (Never delete any element from m_triangles) 
    void DoCutting();

    /// Update the triangle's broken info from the current broken edge
    /// input tid is the in the graphic mesh, with brokenEdge and its idx in m_brokenEdges
    /// return true if the cut is valid and done
    bool cutTriangleFromCurrentBrokenEdge(size_t tid, MeshBrokenEdge & edge, size_t eid);

    /// When generating new sub triangle, add the broken edges info to the new triangles if necessary
    /// input tri is the new sub triangle, edgeId is the broken edge id
    void addBrokenEdgeToNewTriangle(MeshTriangle &tri, size_t edgeId);

    /// handle cutting on one isolated triangle of the mesh
    /// input toolstate, triangle id in graphic mesh and in global lists
    void handleIsolatedTri(std::shared_ptr<ToolState> info, size_t tid, size_t tidGlobal);

    /// Update Topology after DoCutting()
    /// add new vertices with their UVcoords and new triangles
    /// clean the buffer and propagate the changes to Physical(PbdModel) and Visual(vtk) 
    void propagateTopologyChanges();

	/// \brief Get vertices positions (that are edittable)
	StdVectorOfVec3d& getVertexPositions()
	{
		return m_vertexPositions;
	}

    /// \brief Project point to triangle
    /// Compute the barycentric coords of the query point in the query triangle
    /// Return true if the projection is inside the triangle
    bool projectPointToTriangle(Vec3d& queryPoint,
        TriangleArray tri, Vec3f &baryCoords);

    /// \brief Project point to triangle along the given direction
    /// Compute the barycentric coords of the query point in the query triangle
    /// Return true if the projection is inside the triangle
    bool projectPointToTriangleAlongDir(Vec3d& queryPoint,
        TriangleArray tri, Vec3f dir, Vec3f &baryCoords);

    /// \brief Compute distance from the query point to the plane of the query triangle
    /// return the value of the distance, + means query point above the plane, - means below
    double pointToPlaneDistance(Vec3d query, TriangleArray tri);
    double pointToPlaneDistance(Vec3d query, Vec3d pv1, Vec3d pv2, Vec3d pv3);


    /// \brief Find common edge between two triangles
    /// save the results in commonEdge
    bool findCommonEdge(TriangleArray& tri1, TriangleArray& tri2, std::vector<size_t> & commonEdge);

    /// \brief Find common triangle between two triangles
    /// return triangle index in the graphic mesh, or return -1 if not found
    bool findCommonTriangleBetween(TriangleArray& tri1, TriangleArray& tri2, size_t & commonTri);

    /// \brief Check if the edge is on boundary
    /// by checking the number of common neighbor trianlges between vert v1 and vert v2
    /// save the result in commonTris, results tri ids are in the graphic mesh
    bool checkEdgeIsOnBoundary(size_t v1, size_t v2, std::vector<size_t>& commonTris);

    /// \brief Find the local edge id inside the triangle
    int findEdgeInTriangle(size_t v1, size_t v2, TriangleArray& tri);

    /// \brief Find the local vertex id inside the triangle
    int findVertexInTriangle(size_t vert, TriangleArray& tri);

    /// \brief Check if the query edge is already broken during this cut
    /// return index of the edge found in m_brokenEdges
    /// return -1 if not found
    int checkIfEdgeBroken(size_t v1, size_t v2);


    /// \brief Check if the query tri has been splitted from the generateCut()
    /// if not found, return -1. Otherwise return the id in m_splitTriangle
    int checkIfTriBeenSplit(int query);


    /// \brief Check if the area of one trianlge is lower than the threshold
    /// input id is the index of the trianlge in the global triangles list
    bool checkTriangleAreaBelowThreshold(TriangleArray& tri, float threshold);

    /// \brief Find the intersection P between two space line segments P1P2, P3P4
    /// P is the projection on P3P4 if they the two lines dont intersect
    /// return the ratio which is P3P / P3P4
    float intersectBetweenTwoLines(Vec3d &P1, Vec3d &P2, Vec3d &P3, Vec3d &P4);

    /// \brief Find if the plane (pv1, pv2, pv3) will intersect the line segment (ev1, ev2)
    /// return true if intersected
    /// save the intersection coord into &intersect 
    bool intersectPlaneToEdge(Vec3d pv1, Vec3d pv2, Vec3d pv3, Vec3d ev1, Vec3d ev2, float &intersect);

    /// \Define the variables to trace the cut 
    Vec3d toolpos; //current
    int startCutTriId=-1, startCutTriIdGlobal = -1; //the starting triangle of the cut in graphic triangles/ global triangles list
    Vec3f startCutBaryCoords; //the starting point in the triangle
    int preCutTriId = -1; //previous cut triangle Index in the current graphic mesh
    int preCutTriIdGlobal; //previous cut triangle Index in the the global trianlges list
    Vec3f preCutPointCoords; //previous cut point barycentric coords
    int newCutTriIdGlobal; //new cut triangle Index in the global trianlges list 

    //shift the toolPos when two cut trianlges only share vertex neighbor
    bool m_toolPosShifted = false;
    int m_cutTriShiftedTo = -1;

    /// Generate Cut from tool contact data
    /// Algorithm Overview: ...
    bool generateCut(std::shared_ptr<ToolState> info);

    /// find contact pair, for vertex grasping/picking
    bool findToolSurfaceContactPair(std::shared_ptr<ToolState> info, std::tuple<size_t, Vec3d*, Vec3d>& tuple);

    /// return the NewVertexInterpolationData for PbdModel to handle constraints
    inline std::vector< std::tuple<size_t, size_t, float> >& getNewVertexInterpolationData() {
        return m_newVertexInterpolationData;
    }

    /// compute the UV coords for the new vertex on broken edge using intepolation
    Vectorf computeVertexUVCoord(size_t v1, size_t v2, float coord);

    /// check the triangle falls on which side of the plane triangle <v1,v2,v3>
    /// input : query triangle and the two vertices position
    /// return : -1 for one side, 1 for another side, 0 for can not be determined
    int checkTriangleOnWhichSideOfPlane(TriangleArray &tri, const Vec3d & v1, const Vec3d & v2, const Vec3d & v3);

    /// Replace the vertex Id in tri : fromId -> toId 
    void replaceTriVertId(TriangleArray & tri, size_t fromId, size_t toId);

    /// Compute the Cosine of the query vertex angle in the query triangle 
    double computeVertexAngleInTriangle(TriangleArray & tri, size_t vid);

    /// list storing all broken edges
    std::vector<MeshBrokenEdge> m_brokenEdges;

    /// Public Data for Debugging
    std::vector<TriangleArray> d_listUncarvableTris; //Debug Rendering : All uncarvable triangles
    std::array<Vec3d, 2> d_toolPos; //Debug Rendering : cutting blade position at the moment when cut happens

protected:

	std::vector<MeshTriangle> m_triangles; //global tris list (pushes in every tri, never pops out)
	std::vector<MeshSplitTriangle> m_splitTriangle; //split triangles list
	StdVectorOfVec3d m_newVertexPositions; // vector of new vertices positions generated
    StdVectorOfVectorf m_newVertexUVs; // vector of new vertices UV coords generated
    std::vector< std::tuple<size_t, size_t, float> > m_newVertexInterpolationData; // new vertices InterpolationData, <v1, v2, ratio> meaning v = (1-ratio)*v1 + ratio*v2 
	int nbrBrokenEdges = 0; 
    int nbrBrokenEdgesHandled = 0; //number of the broken edges that have been handled in doCutting()
    std::vector< std::array<size_t, 3> > m_adjTrianglesWithVertex; //list of adjacent trianlges pairs with one common vertex
	bool m_newCutGenerated = false;
    bool m_needToUpdatePhysicalModel = false;
    std::map<int, int> mapCurrentTriToTris; //map the triangle Id from current physical mesh to the global triangles list

    //std::vector< std::tuple<size_t, size_t, size_t> > m_replaceTriVertIdList; //list of traingles with the vertex to be updated
    float m_initialElementArea; //Assume the inital mesh is structured and initialElementArea is a constant
    size_t initialNumberVertices;
    int whenToSetUncarvable; //With id above this number, the children triangles will be uncarvable
    float stopCriteriaElementArea = 0.05; //mark uncuttable if the element area smaller than 0.05 of the initial element area

    

};

}
