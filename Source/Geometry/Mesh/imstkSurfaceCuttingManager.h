#pragma once

#include "imstkSurfaceMesh.h"
#include "imstkToolSurfaceInteractionPair.h"
#include <iostream>

namespace imstk
{

 /// MeshSplitTriangle : 
 /// for remeshing the partial cut triangle, which will be splitted into 4 subtriangles 
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

/// MeshBrokenEdge : stores information of one broken edge
struct MeshBrokenEdge 
{
    bool isConnected = true; // false if all its neighbor triangle(s) have been full cut 
    std::vector<size_t> idNeiTri; //idx of the neighbor triangle(s) in the global triangle list
	std::array<size_t, 2> nodeId; //idx of the two original end nodes of this edge 
	std::array<size_t, 2> newNodeId; //idx of the two new nodes
	float brokenCoord;	  //broken position = (1-brokenCoord)*v1 + brokenCoord*v2
    MeshBrokenEdge(){}
	MeshBrokenEdge(size_t id1, size_t id2, float broken = 0.5) :
		brokenCoord(broken)
	{
        nodeId = { id1, id2 };
        idNeiTri.resize(0);
	}
};

/// MeshTriangle : Triangle element with cutting info
struct MeshTriangle 
{
	std::array<size_t, 3> nodeId; // idx of the 3 vertices of this triangle
	bool isBroken;        // if the triangle has been cut
	int numberCuts;		  // num of broken edges, supposed to be 0~2; 2 means full-cut triangle, 1 means partial-cut triangle
	int brokenType;		  // indicating where the cuts are: idx of the uncut edge for full-cut triangle; idx of the cut edge for partial-cut triangle
	std::vector<int> brokenEdgeId; //broken edge id in the m_brokenEdges
	bool doneHandlingCut; // prevent repeated handling cut
    bool canBeCut = true; // not cuttable
    bool tooSmallToCut;   // if area smaller than the threshold
	MeshTriangle(std::array<size_t, 3> verts) : nodeId(verts), isBroken(false), tooSmallToCut(false), numberCuts(0), brokenType(-1)
	{
		doneHandlingCut = false;
        brokenEdgeId.resize(0);
	}

};

///  SurfaceCuttingManager : handling the cutting operation on SurfaceMesh
///  derived from SurfaceMesh, store additional cutting info for a specific SurfaceMesh geometry.
///  the geometry info from SurfaceMesh will be inherited and will actually be used for Physical (PBDModel) and Rendering (vtkRenderer).
///  SurfaceCuttingManager will take inputs from ToolSurfaceInteractionPair,
///  it generates the cuts, does the remeshing, then propagate the topology changes to SurfaceMesh, PbdModel and renderer 
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
    ///     1. Traverse all m_brokenEdges :
    ///			Create 2 new vertices on each edge
	///		2. Handle the special cases (if needed):
	///			Do vertex duplication on Tri-vert-tri pair, then do the remeshing of it
	///		3. Handle regular cases: 
    ///        Go through all m_triangles:
	///			Full-cut: split every triangle that has 2 cuts into 3 subtriangles
	///			Partial-cut: Split every triangle that has 1 cut into 4 subtriangles
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

    /// the variables to model the cut 
    Vec3d toolpos; //current tool pos
    int startCutTriId=-1, startCutTriIdGlobal = -1; //the starting triangle of the cut in graphic triangles/ global triangles list
    Vec3f startCutBaryCoords; //the starting point in the triangle
    int preCutTriId = -1; //previous cut triangle Index in the current graphic mesh
    int preCutTriIdGlobal; //previous cut triangle Index in the the global trianlges list
    Vec3f preCutPointCoords; //previous cut point barycentric coords
    int newCutTriIdGlobal; //new cut triangle Index in the global trianlges list 

    //shift the toolPos to the common neighbor tirnalge when two cut trianlges only share one vertex neighbor
    bool m_toolPosShifted = false;
    int m_cutTriShiftedTo = -1; //idx of the tri that toolpoint has shifted to

    /// generateCut algorithm Overview: 
    /// Generate Cut from tool contact data / toolState
    ///
    bool generateCut(std::shared_ptr<ToolState> info);

    /// find the idx of the vertex for grasping/picking
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

	std::vector<MeshTriangle> m_triangles; //global tris list (pushes in every tri, never pops out), store every triangle
	std::vector<MeshSplitTriangle> m_splitTriangle; //split triangles list
	StdVectorOfVec3d m_newVertexPositions; // vector of new vertices positions generated
    StdVectorOfVectorf m_newVertexUVs; // vector of new vertices UV coords generated
    std::vector< std::tuple<size_t, size_t, float> > m_newVertexInterpolationData;  // new vertices InterpolationData to generate their positions in the initial mesh,
                                                                                    // <v1, v2, ratio> meaning v = (1-ratio)*v1 + ratio*v2 
    int nbrBrokenEdgesHandled = 0; //number of the broken edges that have been handled in doCutting()
    std::vector< std::array<size_t, 3> > m_adjTrianglesWithVertex; //special case: list of adjacent trianlges pairs with one common vertex
	bool m_newCutGenerated = false; //flag: true if the remeshing of the current new cut is done
    bool m_needToUpdatePhysicalModel = false; //tell PBDModel to update physical properties and PBDState vectors
    std::map<int, int> mapCurrentTriToTris; //map the triangle Id from current graphic mesh to the global triangles list

    float m_initialElementArea; //Assume the inital mesh is structured and initialElementArea is a constant
    float stopCriteriaElementArea = 0.05; //mark uncuttable if the element area smaller than 0.05 of the initial element area
    size_t initialNumberVertices;
    int whenToSetUncarvable; //With id above this number, the children triangles will be uncarvable
 

    

};

}
