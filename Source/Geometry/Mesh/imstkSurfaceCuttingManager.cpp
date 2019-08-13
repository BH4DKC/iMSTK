// imstk
#include "imstkSurfaceCuttingManager.h"
namespace imstk
{
    //Helper fucntion: clamp the value
    float clamp(float x, float xmin, float xmax)
    {
        if (x < xmin)
            return xmin;
        else if (x > xmax)
            return xmax;
        else
            return x;
    }

    //Helper fucntion: sgn
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    void
    SurfaceCuttingManager::initializeTriangles(const std::vector<TriangleArray>& triangles)
    {
        if (m_originalNumTriangles == 0)
        {
            m_originalNumTriangles = triangles.size();
            m_maxNumTriangles = (size_t)(m_originalNumTriangles * m_loadFactor);
            m_trianglesVertices.reserve(m_maxNumTriangles);
            m_vertexNormals.reserve(m_maxNumVertices);
            m_vertexTangents.reserve(m_maxNumVertices);
            m_triangles.reserve(m_maxNumTriangles);
            m_topologyChanged = true;
        }

        mapCurrentTriToTris.clear();
        if (triangles.size() <= m_maxNumTriangles)
        {
            m_trianglesVertices = triangles;
            m_topologyChanged = true;
            for (int i = 0; i < triangles.size(); i++)
            {
                m_triangles.push_back(MeshTriangle(triangles[i]));
                mapCurrentTriToTris.insert(std::pair<int, int>(i, i)); //intialize the map to be indentical map
            }
        }
        else
        {
            LOG(WARNING) << "Triangles not set, exceeded maximum number of triangles";
        }
        m_newVertexPositions.resize(0);
        m_newVertexUVs.resize(0);
        m_brokenEdges.resize(0);
        m_splitTriangle.resize(0);
        if (triangles.size() > 0) {
            initialNumberVertices = m_vertexPositions.size();
            whenToSetUncarvable = m_trianglesVertices.size();
            m_initialElementArea = (m_vertexPositions[triangles[0].at(0)] - m_vertexPositions[triangles[0].at(1)])
                .cross(m_vertexPositions[triangles[0].at(0)] - m_vertexPositions[triangles[0].at(2)]).norm();
        }
    }


    Vectorf
    SurfaceCuttingManager::computeVertexUVCoord(size_t v1, size_t v2, float coord)
    {
        Vectorf uv(2);
        if (this->hasPointDataArray(m_defaultTCoords))
        {
            StdVectorOfVectorf* UVs = this->getPointDataArrayEdittable(m_defaultTCoords);
            if (v1 < UVs->size() && v2 < UVs->size())
            {
                uv[0] = (1 - coord) * UVs->at(v1)[0] + coord * UVs->at(v2)[0];
                uv[1] = (1 - coord) * UVs->at(v1)[1] + coord * UVs->at(v2)[1];
            }
            else
                std::cout << "Can not compute UVs on new vertex...\n";
        }
        else
            std::cout << "Error: mesh doesn't have original UVs...\n";
        return uv;
    }

    void
    SurfaceCuttingManager::DoCutting()
    {
        auto oldVertsSize = m_vertexPositions.size();   

        //Create new verts on brokenEdges
        for (auto i = nbrBrokenEdgesHandled; i < m_brokenEdges.size(); i++)
        {
            //add the first new vert, which is close to node[0]
            m_brokenEdges[i].newNodeId[0] = oldVertsSize + m_newVertexPositions.size();
            Vec3d VPos = m_vertexPositions[m_brokenEdges[i].nodeId[0]] * (1.01 - m_brokenEdges[i].brokenCoord)
                + m_vertexPositions[m_brokenEdges[i].nodeId[1]] * (m_brokenEdges[i].brokenCoord - 0.01);
            m_newVertexPositions.push_back(VPos);
            Vectorf uv = computeVertexUVCoord(m_brokenEdges[i].nodeId[0], m_brokenEdges[i].nodeId[1], m_brokenEdges[i].brokenCoord - 0.01);
            m_newVertexUVs.push_back(uv);
            m_newVertexInterpolationData.push_back(std::make_tuple(m_brokenEdges[i].nodeId[0], m_brokenEdges[i].nodeId[1], m_brokenEdges[i].brokenCoord - 0.01));

            //add the second vert
            m_brokenEdges[i].newNodeId[1] = oldVertsSize + m_newVertexPositions.size();
            VPos = m_vertexPositions[m_brokenEdges[i].nodeId[0]] * (0.99 - m_brokenEdges[i].brokenCoord)
                + m_vertexPositions[m_brokenEdges[i].nodeId[1]] * (m_brokenEdges[i].brokenCoord + 0.01);
            m_newVertexPositions.push_back(VPos);
            uv = computeVertexUVCoord(m_brokenEdges[i].nodeId[0], m_brokenEdges[i].nodeId[1], m_brokenEdges[i].brokenCoord + 0.01);
            m_newVertexUVs.push_back(uv);
            m_newVertexInterpolationData.push_back(std::make_tuple(m_brokenEdges[i].nodeId[0], m_brokenEdges[i].nodeId[1], m_brokenEdges[i].brokenCoord + 0.01));

            nbrBrokenEdgesHandled++;
        }

        //Check triangle area and mark too small elements
        for (auto i = 0; i < m_triangles.size(); i++) 
        {
            if (!m_triangles[i].tooSmallToCut)
            {
                m_triangles[i].tooSmallToCut = checkTriangleAreaBelowThreshold(m_triangles[i].nodeId, stopCriteriaElementArea);
            }
        }      

        //Special Case: handle the duplication of vertices
        //Update the neighbor triangles' vertices and broken edges info
            /*1
              |\
              | \
              |	 3\ 
              |	   4\
        ______|______\
      5 \      |      2
         6\    |	
           7\  |	  
              \|
               8*/
        for (auto i = 0; i < m_adjTrianglesWithVertex.size(); i++)
        {
            size_t tri1 = m_adjTrianglesWithVertex[i].at(0);
            size_t tri2 = m_adjTrianglesWithVertex[i].at(1);
            size_t nodeId = m_adjTrianglesWithVertex[i].at(2);

            if (!m_triangles[tri1].doneHandlingCut && !m_triangles[tri2].doneHandlingCut &&
                !m_triangles[tri1].tooSmallToCut && !m_triangles[tri2].tooSmallToCut &&
                m_triangles[tri1].numberCuts == 1 && m_triangles[tri2].numberCuts == 1)
            {  
                //create newNode by duplicating the node position
                size_t newNodeId = m_vertexPositions.size() + m_newVertexPositions.size();             
                m_newVertexPositions.push_back(m_vertexPositions[nodeId]);
                Vectorf uv = computeVertexUVCoord(nodeId, m_brokenEdges[m_triangles[tri1].brokenEdgeId[0]].nodeId[0], 0.01);
                m_newVertexUVs.push_back(uv);
                m_newVertexInterpolationData.push_back(std::make_tuple(nodeId, m_brokenEdges[m_triangles[tri1].brokenEdgeId[0]].nodeId[0], 0.01));

                //create 2 subtriangles in tri 1
                size_t localNodeIdInTri1 = findVertexInTriangle(nodeId, m_triangles[tri1].nodeId);
                size_t v1 = m_brokenEdges[m_triangles[tri1].brokenEdgeId[0]].nodeId[0];
                size_t v2 = m_brokenEdges[m_triangles[tri1].brokenEdgeId[0]].nodeId[1];
                size_t v3 = m_brokenEdges[m_triangles[tri1].brokenEdgeId[0]].newNodeId[0];
                size_t v4 = m_brokenEdges[m_triangles[tri1].brokenEdgeId[0]].newNodeId[1];
                if ( v1 == m_triangles[tri1].nodeId[(localNodeIdInTri1+2) % 3] )
                {
                    v1 = v2;
                    v2 = m_brokenEdges[m_triangles[tri1].brokenEdgeId[0]].nodeId[0];
                    v3 = v4;
                    v4 = m_brokenEdges[m_triangles[tri1].brokenEdgeId[0]].newNodeId[0];
                }


                TriangleArray NewTri[4];
                NewTri[0] = { nodeId, v1, v3 };
                NewTri[1] = { nodeId, v4, v2 };

                //create 2 subtriangles in tri 2
                size_t localNodeIdInTri2 = findVertexInTriangle(nodeId, m_triangles[tri2].nodeId);
                size_t v5 = m_brokenEdges[m_triangles[tri2].brokenEdgeId[0]].nodeId[0];
                size_t v6 = m_brokenEdges[m_triangles[tri2].brokenEdgeId[0]].nodeId[1];
                size_t v7 = m_brokenEdges[m_triangles[tri2].brokenEdgeId[0]].newNodeId[0];
                size_t v8 = m_brokenEdges[m_triangles[tri2].brokenEdgeId[0]].newNodeId[1];
                if (v5 == m_triangles[tri2].nodeId[(localNodeIdInTri2 + 2) % 3])
                {
                    v5 = v6;
                    v6 = m_brokenEdges[m_triangles[tri2].brokenEdgeId[0]].nodeId[0];
                    v7 = v8;
                    v8 = m_brokenEdges[m_triangles[tri2].brokenEdgeId[0]].newNodeId[0];
                }

                NewTri[2] = { nodeId, v5, v7 };
                NewTri[3] = { nodeId, v8, v6 };
                
                //For each neighbor triangle, check on which side if lays of the splitting plane
                auto splitV0 = m_vertexPositions[nodeId] + m_vertexNormals[nodeId]/*Vec3d(0,1,0)*/;
                auto splitV1 = (m_vertexPositions[v1] + m_vertexPositions[v2]) / 2;
                auto splitV2 = (m_vertexPositions[v5] + m_vertexPositions[v6]) / 2;
             
                if (checkTriangleOnWhichSideOfPlane(TriangleArray({ v1,v1,v1 }), splitV0, splitV1, splitV2) == 1) {
                    replaceTriVertId(NewTri[0], nodeId, newNodeId);
                }
                else {
                    replaceTriVertId(NewTri[1], nodeId, newNodeId);
                }

                if (checkTriangleOnWhichSideOfPlane(TriangleArray({ v5,v5,v5 }), splitV0, splitV1, splitV2) == 1) {
                    replaceTriVertId(NewTri[2], nodeId, newNodeId);
                }
                else {
                    replaceTriVertId(NewTri[3], nodeId, newNodeId);
                }


                for (std::set<size_t>::iterator it = m_vertexNeighborTriangles[nodeId].begin(); it != m_vertexNeighborTriangles[nodeId].end(); it++)
                {
                    if (mapCurrentTriToTris[*it] != tri1 && mapCurrentTriToTris[*it] != tri2 && checkTriangleOnWhichSideOfPlane(m_trianglesVertices[*it], splitV0, splitV1, splitV2) == 1)
                    {
                        //std::cout << "Tri " << mapCurrentTriToTris[*it] << " falls on side: +1 \n";
                        replaceTriVertId(m_triangles[mapCurrentTriToTris[*it]].nodeId, nodeId, newNodeId);
                        //update this triangle's broken edge info if its node id needs to be changed 
                        for (auto eId = 0; eId < m_triangles[mapCurrentTriToTris[*it]].brokenEdgeId.size(); eId++)
                        {
                            if (m_brokenEdges[m_triangles[mapCurrentTriToTris[*it]].brokenEdgeId[eId]].nodeId[0] == nodeId)
                            {
                                m_brokenEdges[m_triangles[mapCurrentTriToTris[*it]].brokenEdgeId[eId]].nodeId[0] = newNodeId;
                            }
                            else if (m_brokenEdges[m_triangles[mapCurrentTriToTris[*it]].brokenEdgeId[eId]].nodeId[1] == nodeId)
                            {
                                m_brokenEdges[m_triangles[mapCurrentTriToTris[*it]].brokenEdgeId[eId]].nodeId[1] = newNodeId;
                            }
                        }
                    }
                }
                
                //add subtriangles
                MeshTriangle subtri_0(NewTri[0]);
                if (tri1 >= whenToSetUncarvable) subtri_0.canBeCut = false;
                MeshTriangle subtri_1(NewTri[1]);
                if (tri1 >= whenToSetUncarvable) subtri_1.canBeCut = false;
                MeshTriangle subtri_2(NewTri[2]);
                if (tri2 >= whenToSetUncarvable) subtri_2.canBeCut = false;
                MeshTriangle subtri_3(NewTri[3]);
                if (tri2 >= whenToSetUncarvable) subtri_3.canBeCut = false;
                m_triangles.push_back(subtri_0);
                m_triangles.push_back(subtri_1);
                m_triangles.push_back(subtri_2);
                m_triangles.push_back(subtri_3);
                m_triangles[tri1].doneHandlingCut = true;
                m_triangles[tri2].doneHandlingCut = true;
            }
        }
        m_adjTrianglesWithVertex.clear();

        //For all broken edges, check if they should still be connected or not 
        for (auto i = 0; i < m_brokenEdges.size(); i++)
        {
            if (m_brokenEdges[i].isConnected)
            {
                bool allNeiTrisBeenCut = true;
                for (size_t k = 0; k < m_brokenEdges[i].idNeiTri.size(); k++)
                {
                    //if the neighbor triangle will not be cut
                    if(m_triangles[m_brokenEdges[i].idNeiTri[k]].numberCuts <= 1 && !m_triangles[m_brokenEdges[i].idNeiTri[k]].doneHandlingCut)
                        allNeiTrisBeenCut = false;

                    //else if the cut will not happen on the kth broken edge
                    if (m_triangles[m_brokenEdges[i].idNeiTri[k]].numberCuts == 2 && m_triangles[m_brokenEdges[i].idNeiTri[k]].brokenEdgeId[0] != i
                        && m_triangles[m_brokenEdges[i].idNeiTri[k]].brokenEdgeId[1] != i)
                        allNeiTrisBeenCut = false;
                }
                if (allNeiTrisBeenCut)
                    m_brokenEdges[i].isConnected = false;
            }
        }

        //Create new verts on splitTriangles -- Disabled for now
        //handleSplitTriangles:
        //for (auto & tri : m_splitTriangle)
        //{
        //    if (tri.newNodeId == 0)
        //    {
        //        Vec3d VPos = m_vertexPositions[tri.nodeId[0]] * tri.baryCoords[0] +
        //            m_vertexPositions[tri.nodeId[1]] * tri.baryCoords[1] +
        //            m_vertexPositions[tri.nodeId[2]] * tri.baryCoords[2];
        //        tri.newNodeId = m_vertexPositions.size() + m_newVertexPositions.size();
        //        m_newVertexPositions.push_back(VPos);
        //        m_newVertexUVs.push_back...
        //        m_newVertexInterpolationData.push_back(std::make_tuple(tri.nodeId[0], tri.nodeId[1], 0.5)); //0.5 is default
        //    }
        //}

        //Handle regular cases, Create new sub triangles
        for (auto i = 0; i < m_triangles.size(); i++)
        {                
            //Doesn't generate cut on too small triangles 
            if (m_triangles[i].doneHandlingCut)
            {
                continue;
            }
            else if (m_triangles[i].isBroken && (m_triangles[i].tooSmallToCut || !m_triangles[i].canBeCut) )
            {
                std::cout <<i<<" can't do remeshing on un-cuttable triangles ...\n";
                continue;
            }

            //handle full-cut triangles
            if (m_triangles[i].numberCuts == 2) // Add three sub-triangles
            {
                // edge 2 not broken, create new nodes v3, v4, v5, v6
                if (m_triangles[i].brokenType == 2) 
                {
                    /*1
                      |\
                      |  \
                   (8)|	  3\ e0
                   (7)|	    4\
                      |___6_5_\
                      0	 e1	  2*/

                    size_t e0id, e1id;  //e0 be the edge (v1->v2), e1 be the edge (v2->v0)
                    if ((m_brokenEdges[m_triangles[i].brokenEdgeId[0]].nodeId[0] +
                        m_brokenEdges[m_triangles[i].brokenEdgeId[0]].nodeId[1]) == (m_triangles[i].nodeId[1] + m_triangles[i].nodeId[2]))
                    {
                        e0id = m_triangles[i].brokenEdgeId[0];
                        e1id = m_triangles[i].brokenEdgeId[1];
                    }
                    else
                    {
                        e0id = m_triangles[i].brokenEdgeId[1];
                        e1id = m_triangles[i].brokenEdgeId[0];
                    }
                    //std::cout << "type 2 broken tri edge ids: " << e0id << " " << e1id << " \n";
                    size_t v0 = m_triangles[i].nodeId[0];
                    size_t v1 = m_triangles[i].nodeId[1];
                    size_t v2 = m_triangles[i].nodeId[2];
                    size_t v3 = m_brokenEdges[e0id].newNodeId[0];
                    size_t v4 = m_brokenEdges[e0id].newNodeId[1];
                    if (m_brokenEdges[e0id].nodeId[0] == m_triangles[i].nodeId[2])
                    {
                        v3 = m_brokenEdges[e0id].newNodeId[1];
                        v4 = m_brokenEdges[e0id].newNodeId[0];
                    }
                    size_t v5 = m_brokenEdges[e1id].newNodeId[0];
                    size_t v6 = m_brokenEdges[e1id].newNodeId[1];
                    if (m_brokenEdges[e1id].nodeId[0] == m_triangles[i].nodeId[0])
                    {
                        v5 = m_brokenEdges[e1id].newNodeId[1];
                        v6 = m_brokenEdges[e1id].newNodeId[0];
                    }

                    TriangleArray tri[3];
                    tri[0] = { {v2, v5, v4 } };
                    tri[1] = { {v0, v3, v6 } };
                    tri[2] = { {v0, v1, v3 } };
                    MeshTriangle tri0(tri[0]);                   
                    MeshTriangle tri1(tri[1]);
                    if (i >= whenToSetUncarvable)
                    {
                        tri0.canBeCut = false;
                        tri1.canBeCut = false;
                    }
                    m_triangles.push_back(tri0);
                    m_triangles.push_back(tri1);
                    MeshTriangle tri2(tri[2]);                    
                    int eId = checkIfEdgeBroken(v0, v1);        
                    if (eId != -1)
                    {
                        //special case: if edge2 is also broken:
                        //split tri2 into tri3 and tri4
                        size_t v7, v8;
                        if (m_brokenEdges[eId].nodeId[0] == v0)
                        {
                            v7 = m_brokenEdges[eId].newNodeId[0];
                            v8 = m_brokenEdges[eId].newNodeId[1];
                        }
                        else
                        {
                            v7 = m_brokenEdges[eId].newNodeId[1];
                            v8 = m_brokenEdges[eId].newNodeId[0];
                        }           
                        TriangleArray t3 = { v0, v7, v3 };
                        TriangleArray t4 = { v1, v3, v8 };
                        MeshTriangle tri3(t3);
                        MeshTriangle tri4(t4);
                        if (i >= whenToSetUncarvable)
                        {
                            tri3.canBeCut = false;
                            tri4.canBeCut = false;
                        }
                        m_triangles.push_back(tri3);
                        m_triangles.push_back(tri4);
                    }
                    else
                    {
                        if (i >= whenToSetUncarvable) tri2.canBeCut = false;
                        m_triangles.push_back(tri2);
                    }                   

                    //std::cout << "type 2 broken tri "<< i <<", 3 new tris creted: v0~v6:" << v0 << ", " << v1 << ", " << v2 << ", " << v3 << ", " << v4 << ", " << v5 << ", " << v6 << "\n ";

                }

                if (m_triangles[i].brokenType == 1)
                {
                    /*1
                      |\
                      |  \
                   e2 5	  3\ e0
                      6	    4\
                      |________\
                      0	   8 7  2*/

                    if (m_triangles[i].brokenEdgeId.size() < 2)
                        std::cout << "brokenEdgeId size wrong!" << std::endl;

                    size_t e0id, e2id;  //e0 be the slope edge, e2 be the vertical edge

                    if ((m_brokenEdges[m_triangles[i].brokenEdgeId[0]].nodeId[0] +
                        m_brokenEdges[m_triangles[i].brokenEdgeId[0]].nodeId[1]) == (m_triangles[i].nodeId[1] + m_triangles[i].nodeId[2]))
                    {
                        e0id = m_triangles[i].brokenEdgeId[0];
                        e2id = m_triangles[i].brokenEdgeId[1];
                    }
                    else
                    {
                        e0id = m_triangles[i].brokenEdgeId[1];
                        e2id = m_triangles[i].brokenEdgeId[0];
                    }
                    //std::cout<<"type 1 broken tri edge ids: "<<e0id<<" "<<e2id<<" \n";
                    size_t v0 = m_triangles[i].nodeId[0];
                    size_t v1 = m_triangles[i].nodeId[1];
                    size_t v2 = m_triangles[i].nodeId[2];
                    size_t v3 = m_brokenEdges[e0id].newNodeId[0];
                    size_t v4 = m_brokenEdges[e0id].newNodeId[1];
                    if (m_brokenEdges[e0id].nodeId[0] == m_triangles[i].nodeId[2])
                    {
                        v3 = m_brokenEdges[e0id].newNodeId[1];
                        v4 = m_brokenEdges[e0id].newNodeId[0];
                    }
                    size_t v5 = m_brokenEdges[e2id].newNodeId[0];
                    size_t v6 = m_brokenEdges[e2id].newNodeId[1];
                    if (m_brokenEdges[e2id].nodeId[0] == m_triangles[i].nodeId[0])
                    {
                        v5 = m_brokenEdges[e2id].newNodeId[1];
                        v6 = m_brokenEdges[e2id].newNodeId[0];
                    }

                    TriangleArray tri[3];
                    tri[0] = { {v2, v0, v4 } };
                    tri[1] = { {v0, v6, v4 } };
                    tri[2] = { {v5, v1, v3 } };
                    MeshTriangle tri0(tri[0]);                  
                    MeshTriangle tri1(tri[1]);
                    MeshTriangle tri2(tri[2]);
                    if (i >= whenToSetUncarvable)
                    {
                        tri1.canBeCut = false;
                        tri2.canBeCut = false;
                        tri0.canBeCut = false;
                    }
                    m_triangles.push_back(tri2);
                    m_triangles.push_back(tri1);
                    int eId = checkIfEdgeBroken(v0, v2);
                    if (eId != -1)
                    {
                        //special case: if edge 1 is also broken:
                        //split tri2 into tri3 and tri4
                        size_t v7, v8;
                        if (m_brokenEdges[eId].nodeId[0] == v2)
                        {
                            v7 = m_brokenEdges[eId].newNodeId[0];
                            v8 = m_brokenEdges[eId].newNodeId[1];
                        }
                        else
                        {
                            v7 = m_brokenEdges[eId].newNodeId[1];
                            v8 = m_brokenEdges[eId].newNodeId[0];
                        }
                        TriangleArray t3 = { v2, v7, v4 };
                        TriangleArray t4 = { v0, v4, v8 };
                        MeshTriangle tri3(t3);
                        MeshTriangle tri4(t4);
                        if (i >= whenToSetUncarvable)
                        {
                            tri3.canBeCut = false;
                            tri4.canBeCut = false;
                        }
                        m_triangles.push_back(tri3);
                        m_triangles.push_back(tri4);
                    }
                    else
                    {
                        m_triangles.push_back(tri0);   
                    }           
                    //std::cout << "type 1 broken tri " << i << ", 3 new tris creted: v0~v6:" << v0 << ", " << v1 << ", " << v2 << ", " << v3 << ", " << v4 << ", " << v5 << ", " << v6 << "\n ";

                }

                if (m_triangles[i].brokenType == 0)
                {
                    /*1
                      |\
                      |  \
                   e2 5	   \(7)
                      6	     \(8)
                      |___3_4__\
                     0	  e1    2*/

                    if (m_triangles[i].brokenEdgeId.size() < 2)
                        std::cout << "brokenEdgeId size wrong!" << std::endl;

                    size_t e1id, e2id;
                    if ((m_brokenEdges[m_triangles[i].brokenEdgeId[0]].nodeId[0] +
                        m_brokenEdges[m_triangles[i].brokenEdgeId[0]].nodeId[1]) == (m_triangles[i].nodeId[0] + m_triangles[i].nodeId[2]))
                    {
                        e1id = m_triangles[i].brokenEdgeId[0];
                        e2id = m_triangles[i].brokenEdgeId[1];
                    }
                    else
                    {
                        e1id = m_triangles[i].brokenEdgeId[1];
                        e2id = m_triangles[i].brokenEdgeId[0];
                    }

                    //std::cout << "type 0 broken tri edge ids: " << e1id << " " << e2id << " \n";
                    size_t v0 = m_triangles[i].nodeId[0];
                    size_t v1 = m_triangles[i].nodeId[1];
                    size_t v2 = m_triangles[i].nodeId[2];
                    size_t v3 = m_brokenEdges[e1id].newNodeId[0];
                    size_t v4 = m_brokenEdges[e1id].newNodeId[1];
                    if (m_brokenEdges[e1id].nodeId[0] == m_triangles[i].nodeId[2])
                    {
                        v3 = m_brokenEdges[e1id].newNodeId[1];
                        v4 = m_brokenEdges[e1id].newNodeId[0];
                    }
                    size_t v5 = m_brokenEdges[e2id].newNodeId[0];
                    size_t v6 = m_brokenEdges[e2id].newNodeId[1];
                    if (m_brokenEdges[e2id].nodeId[0] == m_triangles[i].nodeId[0])
                    {
                        v5 = m_brokenEdges[e2id].newNodeId[1];
                        v6 = m_brokenEdges[e2id].newNodeId[0];
                    }

                    TriangleArray tri[3];
                    tri[0] = { {v2, v4, v1 } };
                    tri[1] = { {v1, v4, v5 } };
                    tri[2] = { {v6, v3, v0 } };
                    MeshTriangle tri0(tri[0]);
                    MeshTriangle tri1(tri[1]);
                    MeshTriangle tri2(tri[2]);
                    if (i >= whenToSetUncarvable)
                    {
                        tri0.canBeCut = false;
                        tri1.canBeCut = false;
                        tri2.canBeCut = false;
                    }
                    m_triangles.push_back(tri1);
                    m_triangles.push_back(tri2);
                    int eId = checkIfEdgeBroken(v1, v2);
                    if (eId != -1)
                    {
                        //special case: if edge0 is also broken:
                        //split tri0 into tri3 and tri4
                        size_t v7, v8;
                        if (m_brokenEdges[eId].nodeId[0] == v1)
                        {
                            v7 = m_brokenEdges[eId].newNodeId[0];
                            v8 = m_brokenEdges[eId].newNodeId[1];
                        }
                        else
                        {
                            v7 = m_brokenEdges[eId].newNodeId[1];
                            v8 = m_brokenEdges[eId].newNodeId[0];
                        }
                        TriangleArray t3 = { v1, v7, v4 };
                        TriangleArray t4 = { v2, v4, v7 };
                        MeshTriangle tri3(t3);
                        MeshTriangle tri4(t4);
                        if (i >= whenToSetUncarvable)
                        {
                            tri3.canBeCut = false;
                            tri4.canBeCut = false;
                        }
                        m_triangles.push_back(tri3);
                        m_triangles.push_back(tri4);
                    }
                    else
                    {
                        m_triangles.push_back(tri0);
                    }
                    //std::cout << "3 new tris creted: v0~v6:" << v0 << ", " << v1 << ", " << v2 << ", " << v3 << ", " << v4 << ", " << v5 << ", " << v6 << "\n ";
                }

                m_triangles[i].doneHandlingCut = true;
            }

            //hanlde split triangles
            else if (m_triangles[i].numberCuts == 1)
            {
                int splitTriId = checkIfTriBeenSplit(i);
                //std::cout << "tri " << i << " splitTriId = " << splitTriId << std::endl;
                if (splitTriId == -1)
                {
                    //std::cout << "found one-cut tri " << i << ", put in the waiting list for a full cut to happen... \n";
                    continue; //i->i+1
                }

                if (m_triangles[i].brokenType == 0)//edge (v1-v2) broken
                {
                    /*1
                      |\
                      |  \
                      |	 3\
                      |	 5 4\
                      |________\
                      0		  2*/
                    size_t e0id = m_triangles[i].brokenEdgeId[0];
                    if (m_triangles[i].brokenEdgeId.size() >= 2) // why?
                    {
                        std::cout << "Error: Found 2 broken edges for Tri " << i << ", supposed to have only one ... \n";
                    }
                    else
                    {
                        auto  v0 = m_triangles[i].nodeId[0];
                        auto  v1 = m_triangles[i].nodeId[1];
                        auto  v2 = m_triangles[i].nodeId[2];
                        auto  v3 = m_brokenEdges[e0id].newNodeId[0];
                        auto  v4 = m_brokenEdges[e0id].newNodeId[1];
                        if (m_brokenEdges[e0id].nodeId[0] == m_triangles[i].nodeId[2])
                        {
                            v3 = m_brokenEdges[e0id].newNodeId[1];
                            v4 = m_brokenEdges[e0id].newNodeId[0];
                        }
                        auto  v5 = m_splitTriangle[splitTriId].newNodeId;
                        //std::cout << "brokenType 0 Split Tri, v3~v5: " << v3 << ", " << v4 << ", " << v5 << std::endl;

                        TriangleArray tri[4];
                        tri[0] = { {v0, v1, v5 } };
                        tri[1] = { {v1, v3, v5 } };
                        tri[2] = { {v5, v4, v2 } };
                        tri[3] = { {v0, v5, v2 } };
                        MeshTriangle tri0(tri[0]);
                        MeshTriangle tri1(tri[1]);
                        MeshTriangle tri2(tri[2]);
                        MeshTriangle tri3(tri[3]);
                        m_triangles.push_back(tri0);
                        m_triangles.push_back(tri1);
                        m_triangles.push_back(tri2);
                        m_triangles.push_back(tri3);

                    }

                }

                if (m_triangles[i].brokenType == 1)//edge (v2-v0) broken
                {
                    /*1
                      |\
                      |  \
                      |	 \
                      |	5  \
                      |___4_3__\
                      0		  2*/
                    size_t e0id = m_triangles[i].brokenEdgeId[0];
                    auto  v0 = m_triangles[i].nodeId[0];
                    auto  v1 = m_triangles[i].nodeId[1];
                    auto  v2 = m_triangles[i].nodeId[2];
                    auto  v3 = m_brokenEdges[e0id].newNodeId[0];
                    auto  v4 = m_brokenEdges[e0id].newNodeId[1];
                    if (m_brokenEdges[e0id].nodeId[0] == m_triangles[i].nodeId[0])
                    {
                        v3 = m_brokenEdges[e0id].newNodeId[1];
                        v4 = m_brokenEdges[e0id].newNodeId[0];
                    }
                    auto  v5 = m_splitTriangle[splitTriId].newNodeId;
                    //std::cout << "brokenType 1 Split Tri, v3~v5: " << v3 << ", " << v4 << ", " << v5 << std::endl;
                    TriangleArray tri[4];
                    tri[0] = { {v0, v1, v5 } };
                    tri[1] = { {v1, v2, v5 } };
                    tri[2] = { {v5, v2, v3 } };
                    tri[3] = { {v0, v5, v4 } };
                    MeshTriangle tri0(tri[0]);
                    MeshTriangle tri1(tri[1]);
                    MeshTriangle tri2(tri[2]);
                    MeshTriangle tri3(tri[3]);
                    m_triangles.push_back(tri0);
                    m_triangles.push_back(tri1);
                    m_triangles.push_back(tri2);
                    m_triangles.push_back(tri3);

                }

                if (m_triangles[i].brokenType == 2)//edge (v0-v1) broken
                {
                    /*1
                      |\
                      |  \
                      4	 \
                      3	5  \
                      |________\
                      0		  2*/

                    size_t e0id = m_triangles[i].brokenEdgeId[0];
                    auto  v0 = m_triangles[i].nodeId[0];
                    auto  v1 = m_triangles[i].nodeId[1];
                    auto  v2 = m_triangles[i].nodeId[2];
                    auto  v3 = m_brokenEdges[e0id].newNodeId[0];
                    auto  v4 = m_brokenEdges[e0id].newNodeId[1];
                    if (m_brokenEdges[e0id].nodeId[0] == m_triangles[i].nodeId[1])
                    {
                        v3 = m_brokenEdges[e0id].newNodeId[1];
                        v4 = m_brokenEdges[e0id].newNodeId[0];
                    }
                    auto  v5 = m_splitTriangle[splitTriId].newNodeId;
                    //std::cout << "brokenType 2 Split Tri, v3~v5: " << v3 << ", " << v4 << ", " << v5 << std::endl;
                    TriangleArray tri[4];
                    tri[0] = { {v0, v3, v5 } };
                    tri[1] = { {v1, v5, v4 } };
                    tri[2] = { {v1, v2, v5 } };
                    tri[3] = { {v2, v0, v5 } };
                    MeshTriangle tri0(tri[0]);
                    MeshTriangle tri1(tri[1]);
                    MeshTriangle tri2(tri[2]);
                    MeshTriangle tri3(tri[3]);
                    m_triangles.push_back(tri0);
                    m_triangles.push_back(tri1);
                    m_triangles.push_back(tri2);
                    m_triangles.push_back(tri3);
                }

                m_triangles[i].doneHandlingCut = true;
            }
        }

        //clear all temp data
        //m_brokenEdges.clear();
        //m_splitTriangle.clear();
    }


    bool
    SurfaceCuttingManager::cutTriangleFromCurrentBrokenEdge(size_t tid, MeshBrokenEdge & edge, size_t eid)
    {
        size_t tidGlobal = mapCurrentTriToTris[tid];
        if (m_triangles[tidGlobal].numberCuts < 2 && m_triangles[tidGlobal].canBeCut && !m_triangles[tidGlobal].tooSmallToCut)
        {
            if (!m_triangles[tidGlobal].isBroken)
                m_triangles[tidGlobal].isBroken = true;
            m_triangles[tidGlobal].numberCuts++;

            int localEdgeId = findEdgeInTriangle(edge.nodeId[0], edge.nodeId[1], m_trianglesVertices[tid]);
            if (m_triangles[tidGlobal].numberCuts == 1)
            {
                m_triangles[tidGlobal].brokenType = localEdgeId;
            }
            else
            {
                m_triangles[tidGlobal].brokenType = 3 - m_triangles[tidGlobal].brokenType - localEdgeId;
            }
            m_triangles[tidGlobal].brokenEdgeId.push_back(eid);
            return true;
        }
        else
        {
            std::cout << "Tri " << "[" << tid << ", " << tidGlobal << "] " << "can not be cut again...\n";
            return false;  
        }
    }

    void 
    SurfaceCuttingManager::addBrokenEdgeToNewTriangle(MeshTriangle &tri, size_t edgeId)
    {
        int localEid = findEdgeInTriangle(m_brokenEdges[edgeId].nodeId[0], m_brokenEdges[edgeId].nodeId[1], tri.nodeId);
        if (!tri.isBroken)
            tri.isBroken = true;
        tri.numberCuts++;
        if (tri.numberCuts == 1)
        {
            tri.brokenType = localEid;
        }
        else
        {
            tri.brokenType = 3 - tri.brokenType - localEid;
        }
        tri.brokenEdgeId.push_back(edgeId);
        std::cout << "subtriangle info: " << tri.numberCuts << " " << tri.brokenType << std::endl;
    }

    void 
    SurfaceCuttingManager::replaceTriVertId(TriangleArray & tri, size_t fromId, size_t toId)
    {
        for (size_t i = 0; i < 3; i++)
        {
            if (tri[i] == fromId)
            {
                tri[i] = toId;
                return;
            }        
        }
    }
    
    void 
    SurfaceCuttingManager::propagateTopologyChanges()
    {
        m_TopologyLock.lock();
        size_t nbrElements = m_trianglesVertices.size();
        
        if (m_triangles.size() * 1.5 >= m_maxNumTriangles)
        {
            this->setLoadFactor(this->getLoadFactor() + 1);
        }

        //Update new Vertices UVcoords
        bool hasUVs = this->hasPointDataArray(m_defaultTCoords);
        StdVectorOfVectorf* UVs;
        if (hasUVs)
        {
            UVs = this->getPointDataArrayEdittable(m_defaultTCoords);
            for (auto i = 0; i < m_newVertexUVs.size(); i++)
            {
                UVs->push_back(m_newVertexUVs[i]);
                //std::cout << "new vert uvs: " << m_newVertexUVs[i][0] << " " << m_newVertexUVs[i][1] << std::endl;
            }
        }
        m_newVertexUVs.clear();
        
        //Copy new Vertices to m_vertexPositions
        for (auto i = 0; i < m_newVertexPositions.size(); i++)
        {
            m_vertexPositions.push_back(m_newVertexPositions[i]);
        }           
        m_newVertexPositions.clear();     
        //std::cout << "done propagating... " << this->getPointDataArray(m_defaultTCoords)->size() << " " << this->m_vertexPositions.size() << std::endl;
        
        //Copy new sub triangles to m_trianglesVertices for Visual Rendering
        //Clearing m_trianglesVertices first then copy all non-broken triangles takes O(n) in total
        //But updating m_trianglesVertices one by one will take k*O(n) since each vector.erase() takes O(n) time
        m_trianglesVertices.clear();
        mapCurrentTriToTris.clear();
        d_listUncarvableTris.clear();

        for (auto i = 0; i < m_triangles.size(); i++)
        {
            //keep all non-broken triangles and the ones with partial cuts that has not yet been handled
            if ( m_triangles[i].numberCuts <= 1 && !m_triangles[i].doneHandlingCut && !m_triangles[i].tooSmallToCut)
            {
                mapCurrentTriToTris.insert(std::pair<int, int>(m_trianglesVertices.size(), i)); 

                if (!m_triangles[i].tooSmallToCut && checkTriangleAreaBelowThreshold(m_triangles[i].nodeId, stopCriteriaElementArea))
                    m_triangles[i].tooSmallToCut = true;

                m_trianglesVertices.push_back(m_triangles[i].nodeId);
                if (m_triangles[i].tooSmallToCut || !m_triangles[i].canBeCut)
                    d_listUncarvableTris.push_back(m_triangles[i].nodeId);
            }
        }

        this->computeVertexNeighborTriangles();
        this->computeTrianglesNormals();
        m_topologyChanged = true; //flag to update other componenets of surfaceMesh and visual
        m_needToUpdatePhysicalModel = true;	//flag to update the PbdModel
        m_TopologyLock.unlock();
    }    

    bool
    SurfaceCuttingManager::projectPointToTriangle(Vec3d& queryPoint,
        TriangleArray tri, Vec3f &baryCoords)
    {
        Vec3f triangle_vertex_0 = m_vertexPositions[tri[0]].cast<float>();
        Vec3f triangle_vertex_1 = m_vertexPositions[tri[1]].cast<float>();
        Vec3f triangle_vertex_2 = m_vertexPositions[tri[2]].cast<float>();
        // u=P2−P1
        Vec3f u = triangle_vertex_1 - triangle_vertex_0;
        // v=P3−P1
        Vec3f v = triangle_vertex_2 - triangle_vertex_0;
        // n=u×v
        Vec3f n = u.cross(v);
        // w=P−P1
        Vec3f w = queryPoint.cast<float>() - triangle_vertex_0;
        // Barycentric coordinates of the projection P′of P onto T:
        // γ=[(u×w)⋅n]/n²
        float gamma = u.cross(w).dot(n) / n.dot(n);
        // β=[(w×v)⋅n]/n²
        float beta = w.cross(v).dot(n) / n.dot(n);
        float alpha = 1 - gamma - beta;
        baryCoords = Vec3f(alpha, beta, gamma);
        // The point P′ lies inside T if:
        return ((0 <= alpha) && (alpha <= 1) &&
            (0 <= beta) && (beta <= 1) &&
            (0 <= gamma) && (gamma <= 1));
    }

    bool 
    SurfaceCuttingManager::projectPointToTriangleAlongDir(Vec3d& queryPoint,
        TriangleArray tri, Vec3f dir, Vec3f &baryCoords)
    {
        Vec3f triangle_vertex_0 = m_vertexPositions[tri[0]].cast<float>();
        Vec3f triangle_vertex_1 = m_vertexPositions[tri[1]].cast<float>();
        Vec3f triangle_vertex_2 = m_vertexPositions[tri[2]].cast<float>();
        Vec3f u = triangle_vertex_1 - triangle_vertex_0;
        Vec3f v = triangle_vertex_2 - triangle_vertex_0;
        Vec3f w = queryPoint.cast<float>() - triangle_vertex_0;
        Vec3f l = dir.normalized() * (u.cross(v)).norm();
        if (dir.dot(u.cross(v)) < 0)
            l = -l;
        float gamma = u.cross(w).dot(l) / l.dot(l);
        float beta = w.cross(v).dot(l) / l.dot(l);
        float alpha = 1 - gamma - beta;
        baryCoords = Vec3f(alpha, beta, gamma);
        return ((0 <= alpha) && (alpha <= 1) &&
            (0 <= beta) && (beta <= 1) &&
            (0 <= gamma) && (gamma <= 1));
    }

    double 
    SurfaceCuttingManager::pointToPlaneDistance(Vec3d query, TriangleArray tri)
    {
        Vec3d triangle_vertex_0 = m_vertexPositions[tri[0]];
        Vec3d triangle_vertex_1 = m_vertexPositions[tri[1]];
        Vec3d triangle_vertex_2 = m_vertexPositions[tri[2]];
        Vec3d u = triangle_vertex_1 - triangle_vertex_0;
        Vec3d v = triangle_vertex_2 - triangle_vertex_0;
        Vec3d w = query - triangle_vertex_0;
        return u.cross(v).dot(w) / (u.cross(v)).norm();
    }

    double
    SurfaceCuttingManager::pointToPlaneDistance(Vec3d query, Vec3d pv1, Vec3d pv2, Vec3d pv3)
    {
        Vec3d u = pv2 - pv1;
        Vec3d v = pv3 - pv1;
        Vec3d w = query - pv1;
        return u.cross(v).dot(w) / (u.cross(v)).norm();
    }


    bool 
    SurfaceCuttingManager::findCommonEdge(TriangleArray& tri1, TriangleArray& tri2, std::vector<size_t> & commonEdge)
    {
        commonEdge.resize(0);
        for (auto i : tri1)
        {
            for (auto j : tri2)
            {
                if (i == j)
                    commonEdge.push_back(i);
            }
        }
        if (commonEdge.size() == 2)
            return true;
        else
            return false;
    }

    bool
    SurfaceCuttingManager::findCommonTriangleBetween(TriangleArray& tri1, TriangleArray& tri2, size_t & commonTri)
    {
        std::vector<size_t> commonVert(0);
        for (auto i : tri1)
        {
            for (auto j : tri2)
            {
                if (i == j)
                    commonVert.push_back(i);
            }
        }
        if (commonVert.size() == 1)
        {
            auto neiTris = getVertexNeighborTriangles(commonVert[0]);
            std::vector<size_t> CommonEdge1(0);
            std::vector<size_t> CommonEdge2(0);
            for (std::set<size_t>::iterator it = neiTris.begin(); it != neiTris.end(); it++)
            {
                if (findCommonEdge(m_trianglesVertices[*it], tri1, CommonEdge1) && findCommonEdge(m_trianglesVertices[*it], tri2, CommonEdge2))
                {
                    commonTri = *it;
                    return true;
                }
            }
        }
        return false;
    }

    bool SurfaceCuttingManager::checkEdgeIsOnBoundary(size_t v1, size_t v2, std::vector<size_t>& commonTris)
    {
        if (v1 == v2)
            std::cout << "Error : checkEdgeIsOnBoundary with v1 v2 being the same vertex...\n";
        commonTris.clear();//make sure it is empty
        std::set_intersection(m_vertexNeighborTriangles[v1].begin(), m_vertexNeighborTriangles[v1].end(),
            m_vertexNeighborTriangles[v2].begin(), m_vertexNeighborTriangles[v2].end(), std::back_inserter(commonTris));

        if (commonTris.size() == 2)
            return false;
        else
            return true;
    }

    int 
    SurfaceCuttingManager::findEdgeInTriangle(size_t v1, size_t v2, TriangleArray& tri)
    {
        if (v1 != tri[0] && v2 != tri[0])
            return 0;
        if (v1 != tri[1] && v2 != tri[1])
            return 1;
        if (v1 != tri[2] && v2 != tri[2])
            return 2;
        //std::cout << "findEdgeInTriangle: could not find edge in triangle...\n";
        return -1;
    }

    int 
    SurfaceCuttingManager::findVertexInTriangle(size_t vert, TriangleArray& tri)
    {
        for(size_t i =0; i<3; i++)
            if(vert == tri[i])
                return i;
        std::cout<<"findVertexInTriangle: could not find vertex in triangle...\n";
        return -1;
    }

    int 
    SurfaceCuttingManager::checkIfEdgeBroken(size_t v1, size_t v2)
    {
        for (size_t eid=0 ; eid < m_brokenEdges.size(); eid++)
        {
            if ((m_brokenEdges[eid].nodeId[0] == v1 && m_brokenEdges[eid].nodeId[1] == v2)
                || (m_brokenEdges[eid].nodeId[1] == v1 && m_brokenEdges[eid].nodeId[0] == v2))
            {
                return eid;
            }
        }
        return -1; //no broken edges at all
    }

    int 
    SurfaceCuttingManager::checkIfTriBeenSplit(int query)
    {
        for (auto i = 0; i < m_splitTriangle.size(); i++)
        {
            if (m_splitTriangle[i].nodeId[0] == m_triangles[query].nodeId[0] &&
                m_splitTriangle[i].nodeId[1] == m_triangles[query].nodeId[1] &&
                m_splitTriangle[i].nodeId[2] == m_triangles[query].nodeId[2])
                return i;
        }
        return -1; // not found
    }

    bool 
    SurfaceCuttingManager::checkTriangleAreaBelowThreshold(TriangleArray& tri, float threshold)
    {
        Vec3f triangle_vertex_0 = m_vertexPositions[tri[0]].cast<float>();
        Vec3f triangle_vertex_1 = m_vertexPositions[tri[1]].cast<float>();
        Vec3f triangle_vertex_2 = m_vertexPositions[tri[2]].cast<float>();
        
        float area = (triangle_vertex_0 - triangle_vertex_1).cross(triangle_vertex_0 - triangle_vertex_2).norm();
        if (area / m_initialElementArea < threshold)
            return true;
        else
            return false;
    }

    float
    SurfaceCuttingManager::intersectBetweenTwoLines(Vec3d &P1, Vec3d &P2, Vec3d &P3, Vec3d &P4) 
    {
        Vec3d E31 = P1 - P3;
        Vec3d E32 = P2 - P3;
        Vec3d E34 = P4 - P3;
        double h1 = E31.dot(E34) / E34.squaredNorm();
        double h2 = E32.dot(E34) / E34.squaredNorm();
        double ratio = E31.cross(E34).norm() / (E31.cross(E34).norm() + E32.cross(E34).norm());
        return h1 + (h2 - h1)*ratio;
    }

    bool 
    SurfaceCuttingManager::intersectPlaneToEdge(Vec3d pv1, Vec3d pv2, Vec3d pv3, Vec3d ev1, Vec3d ev2, float &intersect)
    {
        float d1 = (float)pointToPlaneDistance(ev1, pv1, pv2, pv3);
        float d2 = (float)pointToPlaneDistance(ev2, pv1, pv2, pv3);
        if (d1*d2 > 0)
            return false;
        else
        {
            intersect = abs(d1) / (abs(d1) + abs(d2));
            return true;
        }

    }


    bool 
    SurfaceCuttingManager::generateCut(std::shared_ptr<ToolState> info)
    {
        bool DEBUG = false; //print out debugging info
        double lengthCuttingBlade = (info->toolTipEndPos - info->toolTipStartPos).norm();
        int numberPointsToCut = 30; ///>TODO:: Adaptive step-length, this number should be determined by lengthBlade / averageElementEdgeLength     
        d_toolPos[0] = info->toolTipStartPos; 
        d_toolPos[1] = info->toolTipEndPos;
        
        Vec3f toolUpDir = ((info->toolDir).cross(info->cutPlaneNormal)).cast<float>();/// this should be the tool's up (or down) direction
        //if (DEBUG) std::cout << "tool Dir: " << info->toolDir << ", cutPlaneNormal: " << info->cutPlaneNormal <<"toolUp: "<< toolUpDir << std::endl;
        //Get the candidate triangles list
        //TODO: this should be improved by using better tool collision algorithm or data structures)
        //But for now, just update the candidateTris to include all cuttable graphic triangles
        std::set<size_t> candidateTris(info->triId);    
        for (auto triId = 0; triId < m_trianglesVertices.size(); triId++)
        {
            if (m_triangles[mapCurrentTriToTris[triId]].canBeCut && !m_triangles[mapCurrentTriToTris[triId]].tooSmallToCut)
                candidateTris.insert(triId);
        }

        //Uniform sampling of the cut points along the cutting blade
        for (auto idCut=0; idCut< numberPointsToCut; idCut++)
        {
            Vec3f baryCoords; //baryCoords of current Cut Point for the query triangle
            if (m_toolPosShifted) //tool cut pos has been shifted already
            {
                m_toolPosShifted = false;
            }
            else {//increment the tooltip pos along the blade               
                toolpos = info->toolTipStartPos + (info->toolTipEndPos - info->toolTipStartPos) * idCut / numberPointsToCut;
            }

            // if cut point in the same triangle 
            // then we don't need to query all candi triangles
            // Need to check the cut crosses the boundary
            if (preCutTriId != -1 && projectPointToTriangle(toolpos, m_trianglesVertices[preCutTriId], baryCoords) &&
                projectPointToTriangleAlongDir(toolpos, m_trianglesVertices[preCutTriId], toolUpDir, baryCoords)
                && abs(pointToPlaneDistance(toolpos, m_trianglesVertices[preCutTriId])) < 0.8)
            {
                //if (DEBUG) std::cout << "tool stays in preTri [" << preCutTriId << "," << preCutTriIdGlobal << "]\n";
                //Check if the cut breaks any boundary edge
                for (size_t i = 0; i < 3; i++)
                {
                    Vec3d edgeDir = ( m_vertexPositions[m_trianglesVertices[preCutTriId].at((i + 1) % 3)] -
                        m_vertexPositions[m_trianglesVertices[preCutTriId].at((i + 2) % 3)] ).normalized();
                    double angleToolAndEdge = edgeDir.dot(info->toolDir);
                    float cutPos=-1;
                    intersectPlaneToEdge(info->toolTipStartPos, toolpos, info->toolTipStartPos+toolUpDir.cast<double>(), m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[(i + 1) % 3]],
                        m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[(i + 2) % 3]], cutPos);
                    if (/*baryCoords[i] < 0.25 && abs(angleToolAndEdge) < 0.6 && */cutPos<1 && cutPos>0) //edge i should break
                    {  
                        cutPos = clamp(cutPos, 0.1, 0.9);
                        MeshBrokenEdge e(m_triangles[preCutTriIdGlobal].nodeId[(i + 1) % 3], m_triangles[preCutTriIdGlobal].nodeId[(i + 2) % 3], cutPos);
                        if ( checkIfEdgeBroken(e.nodeId[0], e.nodeId[1]) == -1)
                        {
                            std::vector<size_t> commonTris;
                            if ( checkEdgeIsOnBoundary(e.nodeId[0], e.nodeId[1], commonTris) && m_triangles[preCutTriIdGlobal].numberCuts < 2 )
                            {
                                
                                if (cutTriangleFromCurrentBrokenEdge(preCutTriId, e, m_brokenEdges.size()))
                                {
                                    e.idNeiTri.push_back(preCutTriIdGlobal);
                                    m_brokenEdges.push_back(e);
                                    if (DEBUG) std::cout << "new boundary brokenEdge info: " << m_brokenEdges.size() << "th edge, v1=" << e.nodeId[0] << ", v2=" << e.nodeId[1] << ", cPos=" << cutPos << "angleToolAndEdge=" << angleToolAndEdge << std::endl;
                                }      
                            }
                            break;
                        }
                    }
                }
                continue; //to check the next cut point
            }

            //else: then query each triangle from the tool collision information 
            for (std::set<size_t>::iterator it = candidateTris.begin(); it != candidateTris.end(); it++)
            {            
                //find if tooltip projection inside the triangle *it
                if (projectPointToTriangle(toolpos, m_trianglesVertices[*it], baryCoords) &&
                    projectPointToTriangleAlongDir(toolpos, m_trianglesVertices[*it], toolUpDir, baryCoords) &&
                    abs(pointToPlaneDistance(toolpos, m_trianglesVertices[*it])) < 0.8)
                {
                    newCutTriIdGlobal = mapCurrentTriToTris[*it];
                    if (!m_triangles[newCutTriIdGlobal].canBeCut || m_triangles[newCutTriIdGlobal].tooSmallToCut)
                    {
                        std::cout << "Tri : " << newCutTriIdGlobal << " is not cuttable ...\n";
                        continue;
                    }
                    // check cutting angle
                    float cosCutAngle = getTriangleNormal(*it).dot(info->cutPlaneNormal);
                    if (std::abs(cosCutAngle) > 0.96) // cut angle < 25 degrees
                    {
                        if (DEBUG) {
                            std::cout << " blade angle " << std::abs(cosCutAngle) << " doesn't meet the cutting angle condition...\n" << std::endl;
                        }
                        break;
                    }
                    if (startCutTriId == -1 || startCutTriIdGlobal== -1)
                    {
                        startCutTriId = *it;
                        startCutTriIdGlobal = newCutTriIdGlobal;
                        startCutBaryCoords = baryCoords;
                    }
                    //if this is the start of the cut
                    if (preCutTriId == -1) 
                    {
                        preCutTriId = *it;
                        preCutTriIdGlobal = newCutTriIdGlobal;
                        preCutPointCoords = baryCoords;
                        if(DEBUG) std::cout << "new cut starts...\n preId: " << *it << ", " << preCutTriIdGlobal << "\n";
                        if (m_triangles[preCutTriIdGlobal].numberCuts >= 2)
                        {
                            std::cout << "preCut triangle "<< preCutTriId<<" already has two cuts...\n ";
                            break;
                        }
                        m_triangles[preCutTriIdGlobal].isBroken = true;
                        startCutBaryCoords = baryCoords;
                    }

                    //tool staying in the same tri
                    else if (preCutTriIdGlobal == mapCurrentTriToTris[*it]) 
                    {
                        if (preCutTriId != *it)
                            preCutTriId = *it;
                        preCutPointCoords = baryCoords;
                    }

                    else //tooltip go into a new triangle, udpate the broken info of the two triangle
                    {
                        if (DEBUG) std::cout << "tooltip goto tri [" << *it << "," << newCutTriIdGlobal << "]\n";
                        std::vector<size_t> brokenEdge(0); // common edge between the pre and new trianlge
                        size_t commonTri; // common neighbor triangle between the pre and new trianlge
                        
                        //found one common edge to be cut
                        if (findCommonEdge(m_trianglesVertices[preCutTriId], m_trianglesVertices[*it], brokenEdge))
                        {
                            // check if the brokenEdge is already broken
                            if (checkIfEdgeBroken(brokenEdge[0], brokenEdge[1]) != -1)
                            {
                                std::cout << "can not cut the broken edge again... "<< brokenEdge[0]<<", "<< brokenEdge[1]<<std::endl;
                                //do nothing but reset the preCutri pointer
                                preCutTriId = *it;
                                preCutTriIdGlobal = newCutTriIdGlobal; 
                                preCutPointCoords = baryCoords;
                                break;
                            }                     

                            //Find the broken Position of the broken edge
                            Vec3d P1 = preCutPointCoords[0] * m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[0]] +
                                preCutPointCoords[1] * m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[1]] +
                                preCutPointCoords[2] * m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[2]];
                            Vec3d P2 = baryCoords[0] * m_vertexPositions[m_triangles[newCutTriIdGlobal].nodeId[0]] +
                                baryCoords[1] * m_vertexPositions[m_triangles[newCutTriIdGlobal].nodeId[1]] +
                                baryCoords[2] * m_vertexPositions[m_triangles[newCutTriIdGlobal].nodeId[2]];
                            float cutPos = intersectBetweenTwoLines(P1, P2, m_vertexPositions[brokenEdge[0]], m_vertexPositions[brokenEdge[1]]);                          

                            //clamp the broken Position 
                            cutPos = clamp(cutPos, 0.1, 0.9);

                            //Create new broken edge
                            MeshBrokenEdge e(brokenEdge[0], brokenEdge[1], cutPos);

                            if (m_triangles[preCutTriIdGlobal].numberCuts < 2 || m_triangles[newCutTriIdGlobal].numberCuts < 2)
                            {
                                // update broken info of the two triangles
                                if (cutTriangleFromCurrentBrokenEdge(preCutTriId, e, m_brokenEdges.size()))
                                    e.idNeiTri.push_back(preCutTriIdGlobal);
                                if (cutTriangleFromCurrentBrokenEdge(*it, e, m_brokenEdges.size()))
                                    e.idNeiTri.push_back(newCutTriIdGlobal);
                                //std::cout << "preCutTri Global id " << preCutTriIdGlobal << ", nbrCuts = " << m_triangles[preCutTriIdGlobal].numberCuts << " type = " << m_triangles[preCutTriIdGlobal].brokenType << std::endl;
                                //std::cout << "newCutTri Global id " << newCutTriIdGlobal << ", nbrCuts = " << m_triangles[newCutTriIdGlobal].numberCuts << " type = " << m_triangles[newCutTriIdGlobal].brokenType << std::endl; 
                                if (e.idNeiTri.size() > 0)
                                {
                                    m_brokenEdges.push_back(e);
                                    if (DEBUG) std::cout << "new brokenEdge info: " << m_brokenEdges.size() << "th edge, v1=" << e.nodeId[0] << ", v2=" << e.nodeId[1] << ", cPos=" << cutPos << std::endl;
                                }
                            }

                            // update the preCutTri pointer and clear the new one
                            preCutTriId = *it;
                            preCutTriIdGlobal = newCutTriIdGlobal;
                            preCutPointCoords = baryCoords;
                            break; 
                        }

                        //special case -- triangles has common triangle in between -> shift the cut
                        else if (findCommonTriangleBetween(m_trianglesVertices[preCutTriId], m_trianglesVertices[*it], commonTri) )
                        {
                            if (DEBUG) std::cout << "found the three-adj-tri pair: " << preCutTriId <<" "<<*it<< " "<< commonTri << std::endl;
                            //Move the tooltip into this tri and continue cutting
                            m_toolPosShifted = true;
                            m_cutTriShiftedTo = commonTri;
                            //the toolpos is moved to neighbor triangle center,
                            Vec3d centerCommonTri = m_vertexPositions[m_trianglesVertices[commonTri].at(0)] / 3 +
                                m_vertexPositions[m_trianglesVertices[commonTri].at(1)] / 3 +
                                m_vertexPositions[m_trianglesVertices[commonTri].at(2)] / 3;
                            Vec3d centerPreCutTri = m_vertexPositions[m_trianglesVertices[preCutTriId].at(0)] / 3 +
                                m_vertexPositions[m_trianglesVertices[preCutTriId].at(1)] / 3 +
                                m_vertexPositions[m_trianglesVertices[preCutTriId].at(2)] / 3;
                            Vec3d centerCurCutTri = m_vertexPositions[m_trianglesVertices[*it].at(0)] / 3 +
                                m_vertexPositions[m_trianglesVertices[*it].at(1)] / 3 +
                                m_vertexPositions[m_trianglesVertices[*it].at(2)] / 3;
                            toolpos = centerCommonTri / 5 + centerPreCutTri * 2/5 + centerCurCutTri * 2/5;
                        }

                        //special case -- adjacent triangle with only one common vertex in between -> duplicate the common vertex
                        else if (brokenEdge.size() == 1 && preCutTriIdGlobal < m_originalNumVertices && 
                            newCutTriIdGlobal < m_originalNumVertices)
                        {
                            // check the adjacent triangles if the angle around the common vertex equals
                            double angle1 = computeVertexAngleInTriangle(m_triangles[preCutTriIdGlobal].nodeId, brokenEdge[0]);
                            double angle2 = computeVertexAngleInTriangle(m_triangles[newCutTriIdGlobal].nodeId, brokenEdge[0]);
                            if (abs(angle1 - angle2) < 0.02)
                            {
                                if (DEBUG) std::cout << "found adjacent Triangles With Vertex pair: " << preCutTriIdGlobal << " " << newCutTriIdGlobal << " " << brokenEdge[0] << " will do duplication... "<< abs(angle1 - angle2) <<std::endl;
                                std::array<size_t, 3> pair = { preCutTriIdGlobal, newCutTriIdGlobal, brokenEdge[0] };
                                m_adjTrianglesWithVertex.push_back(pair);
                                Vec3d PreCutPointPos = preCutPointCoords[0] * m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[0]] +
                                    preCutPointCoords[1] * m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[1]] + preCutPointCoords[2] * m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[2]];
                                Vec3d NewCutPointPos = baryCoords[0] * m_vertexPositions[m_triangles[newCutTriIdGlobal].nodeId[0]] +
                                    baryCoords[1] * m_vertexPositions[m_triangles[newCutTriIdGlobal].nodeId[1]] + baryCoords[2] * m_vertexPositions[m_triangles[newCutTriIdGlobal].nodeId[2]];

                                //handle the preCutTri , ignore the other cutting info on this tri
                                if (!m_triangles[preCutTriIdGlobal].tooSmallToCut && m_triangles[preCutTriIdGlobal].numberCuts < 2)
                                {
                                    int EdgeId = findVertexInTriangle(brokenEdge[0], m_triangles[preCutTriIdGlobal].nodeId);
                                    float cutPos = intersectBetweenTwoLines(NewCutPointPos, PreCutPointPos, m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[(EdgeId + 1) % 3]],
                                        m_vertexPositions[m_triangles[preCutTriIdGlobal].nodeId[(EdgeId + 2) % 3]]);
                                    // if the intersection is on the edge and the edge is not broken, then break it
                                    if (EdgeId != -1 && cutPos>0 &&cutPos<1 &&checkIfEdgeBroken(m_triangles[preCutTriIdGlobal].nodeId[(EdgeId + 1) % 3], m_triangles[preCutTriIdGlobal].nodeId[(EdgeId + 2) % 3]) == -1 )
                                    {
                                        //float cutPos = preCutPointCoords[(EdgeId + 2) % 3] / (preCutPointCoords[(EdgeId + 2) % 3] + preCutPointCoords[(EdgeId + 1) % 3]);
                                        cutPos = clamp(cutPos, 0.1, 0.9);
                                        MeshBrokenEdge e(m_triangles[preCutTriIdGlobal].nodeId[(EdgeId + 1) % 3], m_triangles[preCutTriIdGlobal].nodeId[(EdgeId + 2) % 3], cutPos);
                                        std::vector<size_t> brokenEdgeNeiTris;
                                        if (!checkEdgeIsOnBoundary(e.nodeId[0], e.nodeId[1], brokenEdgeNeiTris))
                                        {
                                            size_t neighborTriId = brokenEdgeNeiTris[0] == preCutTriId ? brokenEdgeNeiTris[1] : brokenEdgeNeiTris[0];
                                            e.idNeiTri.push_back(mapCurrentTriToTris[neighborTriId]);
                                            //update the neighbor triangle
                                            cutTriangleFromCurrentBrokenEdge(neighborTriId, e, m_brokenEdges.size());
                                        }
                                        m_triangles[preCutTriIdGlobal].isBroken = true;                                    
                                        m_triangles[preCutTriIdGlobal].numberCuts = 1;
                                        m_triangles[preCutTriIdGlobal].brokenType = EdgeId;
                                        m_triangles[preCutTriIdGlobal].brokenEdgeId.resize(0);
                                        m_triangles[preCutTriIdGlobal].brokenEdgeId.push_back(m_brokenEdges.size());
                                        m_brokenEdges.push_back(e);
                                        //reset the startCutId if it is the preCutTri
                                        if (DEBUG) std::cout << "new brokenEdge info: " << m_brokenEdges.size() << "th edge, v1=" << e.nodeId[0] << ", v2=" << e.nodeId[1] << ", cPos=" << cutPos << std::endl;
                                    }
                                    //std::cout << "preCutTri Info: cuts, type, edgeId = " << m_triangles[preCutTriIdGlobal].numberCuts << ", " << m_triangles[preCutTriIdGlobal].brokenType << ", "
                                        //<< m_triangles[preCutTriIdGlobal].brokenEdgeId[0]<<std::endl;
                                }

                                //handle the newCutTri , ignore the other cutting info on this tri
                                if (!m_triangles[newCutTriIdGlobal].tooSmallToCut && m_triangles[newCutTriIdGlobal].numberCuts < 2)
                                {
                                    int EdgeId = findVertexInTriangle(brokenEdge[0], m_triangles[newCutTriIdGlobal].nodeId);
                                    float cutPos = intersectBetweenTwoLines(PreCutPointPos, NewCutPointPos, m_vertexPositions[m_triangles[newCutTriIdGlobal].nodeId[(EdgeId + 1) % 3]],
                                        m_vertexPositions[m_triangles[newCutTriIdGlobal].nodeId[(EdgeId + 2) % 3]]);
                                    if ( EdgeId != -1 && cutPos > 0 && cutPos < 1 && checkIfEdgeBroken(m_triangles[newCutTriIdGlobal].nodeId[(EdgeId + 1) % 3], m_triangles[newCutTriIdGlobal].nodeId[(EdgeId + 2) % 3]) == -1 )
                                    {
                                        //float cutPos = baryCoords[(EdgeId + 2) % 3] / (baryCoords[(EdgeId + 2) % 3] + baryCoords[(EdgeId + 1) % 3]);
                                        cutPos = clamp(cutPos, 0.1, 0.9);
                                        MeshBrokenEdge e(m_triangles[newCutTriIdGlobal].nodeId[(EdgeId + 1) % 3], m_triangles[newCutTriIdGlobal].nodeId[(EdgeId + 2) % 3], cutPos);
                                        std::vector<size_t> brokenEdgeNeiTris;
                                        if (!checkEdgeIsOnBoundary(e.nodeId[0], e.nodeId[1], brokenEdgeNeiTris))
                                        {
                                            size_t neighborTriId = brokenEdgeNeiTris[0] == *it ? brokenEdgeNeiTris[1] : brokenEdgeNeiTris[0];
                                            e.idNeiTri.push_back(mapCurrentTriToTris[neighborTriId]);
                                            //update the neighbor triangle
                                            cutTriangleFromCurrentBrokenEdge(neighborTriId, e, m_brokenEdges.size());

                                        }
                                        m_triangles[newCutTriIdGlobal].isBroken = true;
                                        m_triangles[newCutTriIdGlobal].numberCuts = 1;
                                        m_triangles[newCutTriIdGlobal].brokenType = EdgeId;
                                        m_triangles[newCutTriIdGlobal].brokenEdgeId.resize(0);
                                        m_triangles[newCutTriIdGlobal].brokenEdgeId.push_back(m_brokenEdges.size());
                                        m_brokenEdges.push_back(e);
                                        //reset the startCutId if it is the newCutTri
                                        if (newCutTriIdGlobal == startCutTriIdGlobal)
                                            startCutTriIdGlobal = -1;
                                        if (DEBUG) std::cout << "line 1300 new brokenEdge info: " << m_brokenEdges.size() << "th edge, v1=" << e.nodeId[0] << ", v2=" << e.nodeId[1] << ", cPos=" << cutPos << std::endl;
                                    }
                                }
                            }
                            // reset pointers and stop handling new cuts on the same triangles
                            // the purpose is to make sure these triangles can not be modified again
                            if (startCutTriIdGlobal == newCutTriIdGlobal || startCutTriIdGlobal == preCutTriIdGlobal)
                                startCutTriIdGlobal = -1;
                            preCutTriId = -1;
                            idCut = numberPointsToCut;
                            break;
                        }

                        else //the two triangles are not connected
                        {   
                            //special case: that two neighbor tris are not conformal 
                            if (brokenEdge.size() == 1)
                            {
                                std::cout << "Warning: non-conformal connected triangles detected... " << std::endl;                                
                            }
                            else
                            {
                                std::cout << "Warning: isolated cut detected... "<< std::endl;
                            }
                            //handleIsolatedTri(info, preCutTriId, preCutTriIdGlobal);
                            //handleIsolatedTri(info, *it, newCutTriIdGlobal);
                            /*handleIsolatedTri(info, startCutTriId, startCutTriIdGlobal);
                            startCutTriId = -1;
                            startCutTriIdGlobal = -1;*/
                            preCutTriId = -1;
                            preCutTriIdGlobal = -1;                           
                            break;    
                        }
                    } //findCommonEdge between pre and *it

                    //break; //handle one tri at a time. break the for loop
                }
            } //foreach query tri

        } //foreach cut point on blade

        //Handle the starting triangle of the cut
        //reset all pointers
        if (DEBUG) std::cout << "Handling the starting triangle... "<< startCutTriIdGlobal<<"\n";
        handleIsolatedTri(info, startCutTriId, startCutTriIdGlobal);
        startCutTriId = -1;
        startCutTriIdGlobal = -1;
        preCutTriId = -1;
        preCutTriIdGlobal = -1;
        //Check if there is topology change waitting to be handled/propagate
        if (m_brokenEdges.size() - nbrBrokenEdgesHandled >= 1) // when there is at least one new broken edge
        {
            std::cout << "Cut detected, now start remeshing ...\n";
            DoCutting();
            m_newCutGenerated = true;
            return true;
        }

        return false;
    }

    void 
    SurfaceCuttingManager::handleIsolatedTri(std::shared_ptr<ToolState> info, size_t tid, size_t tidGlobal)
    {
        bool DEBUG = false;
        if (tidGlobal != -1 && m_triangles[tidGlobal].numberCuts < 2 && !m_triangles[tidGlobal].doneHandlingCut 
            && m_triangles[tidGlobal].canBeCut &&!m_triangles[tidGlobal].tooSmallToCut)
        {
            //check the 3 edges, break if cutting blade intersects
            Vec3d toolUpDir = (info->toolDir).cross(info->cutPlaneNormal);
            Vec3d edgeDir[3];
            bool isEdgeOnBoundary[3];
            float cutPos[3] = {-1,-1,-1};
            float angleToolAndEdge[3];
            std::vector<int> edgeBroken(3);
            std::vector<size_t> tempTris;

            bool thisTriIsIsolated = false;
            for (int i = 0; i < 3; i++)
            {
                edgeDir[i] = (m_vertexPositions[m_triangles[tidGlobal].nodeId[(i + 1) % 3]] -
                    m_vertexPositions[m_triangles[tidGlobal].nodeId[(i + 2) % 3]]).normalized();
                angleToolAndEdge[i] = edgeDir[i].dot(info->toolDir);
                
                intersectPlaneToEdge(info->toolTipStartPos, info->toolTipEndPos, info->toolTipStartPos + toolUpDir,
                    m_vertexPositions[m_triangles[tidGlobal].nodeId[(i + 1) % 3]], m_vertexPositions[m_triangles[tidGlobal].nodeId[(i + 2) % 3]], cutPos[i]);
                /*cutPos[i] = intersectBetweenTwoLines(info->toolTipStartPos, info->toolTipEndPos, m_vertexPositions[m_triangles[tidGlobal].nodeId[(i + 1) % 3]],
                    m_vertexPositions[m_triangles[tidGlobal].nodeId[(i + 2) % 3]]);*/
                
                isEdgeOnBoundary[i] = checkEdgeIsOnBoundary(m_triangles[tidGlobal].nodeId[(i + 1) % 3], m_triangles[tidGlobal].nodeId[(i + 2) % 3], tempTris);
                if (cutPos[i] < 1 && cutPos[i] > 0 && abs(angleToolAndEdge[i])<0.96)
                    edgeBroken[i] = 1;
            }
            if(DEBUG) std::cout << "isolated tri " << tidGlobal << " has info: brokenEdge" << edgeBroken[0] << edgeBroken[1] << edgeBroken[2] <<" angleToolEdge: "<< angleToolAndEdge[0]
                << angleToolAndEdge[1]<< angleToolAndEdge[2]<<"; cutPos:"<< cutPos[0]<< cutPos[1]<< cutPos[2]<< " ; isOnBoundary:" << isEdgeOnBoundary[0] << isEdgeOnBoundary[1] << isEdgeOnBoundary[2] << std::endl;
            if (edgeBroken[0] + edgeBroken[1] + edgeBroken[2] == 3)
            {
                std::cout << "Warning : special case detected -- remove this triangle since it has 3 edges cut by the blade...\n";
                m_triangles[tidGlobal].doneHandlingCut = true;
            }
            else if (edgeBroken[0] + edgeBroken[1] + edgeBroken[2] == 2) // this triangle needs to be cut
            {
                //Special case : isolated triangle
                if (isEdgeOnBoundary[0] + isEdgeOnBoundary[1] + isEdgeOnBoundary[2] == 2)
                {
                    m_triangles[tidGlobal].doneHandlingCut = true; //remove
                }  
                else if (isEdgeOnBoundary[0] + isEdgeOnBoundary[1] + isEdgeOnBoundary[2] == 1) 
                {
                    //reset this triangle
                    m_triangles[tidGlobal].numberCuts = 0;
                    m_triangles[tidGlobal].brokenEdgeId.resize(0);
                    m_triangles[tidGlobal].brokenType = -1;
                    thisTriIsIsolated = true;
                    if (DEBUG) std::cout << "Tri " << tidGlobal << "is isolated...\n";
                }

                //Special case : when a partial broken triangle get a full cut on the other 2 edges, reset this triangle first
                /*if(m_triangles[tidGlobal].numberCuts ==1 && !edgeBroken[m_triangles[tidGlobal].brokenType]) { }*/              
                
                for (int i = 0; i < 3; i++)
                {
                    if (edgeBroken[i])
                    {
                        MeshBrokenEdge e(m_triangles[tidGlobal].nodeId[(i + 1) % 3], m_triangles[tidGlobal].nodeId[(i + 2) % 3], clamp(cutPos[i], 0.1, 0.9));
                        if (checkIfEdgeBroken(e.nodeId[0], e.nodeId[1]) == -1)
                        {
                            std::vector<size_t> commonTris;
                            if (!checkEdgeIsOnBoundary(e.nodeId[0], e.nodeId[1], commonTris))
                            {
                                //not on boundary, should cut the neighbor triangle
                                size_t neighborTriId = commonTris[0] == tid ? commonTris[1] : commonTris[0];
                                size_t neighborTriIdGlobal = mapCurrentTriToTris[neighborTriId];

                                //update the neighbor triangle
                                if (cutTriangleFromCurrentBrokenEdge(neighborTriId, e, m_brokenEdges.size()))
                                    e.idNeiTri.push_back(neighborTriIdGlobal);
                                if (DEBUG) std::cout << "isolated tri "<<tidGlobal<< " with neiTri "<<neighborTriIdGlobal<< " cut by " << m_brokenEdges.size()+1 << " th edge, v1=" << e.nodeId[0]
                                    << ", v2=" << e.nodeId[1] << ", cPos=" << cutPos[i] << std::endl;
                            }
                            if (cutTriangleFromCurrentBrokenEdge(tid, e, m_brokenEdges.size()))
                                e.idNeiTri.push_back(tidGlobal);
                            if (e.idNeiTri.size() > 0)
                            {
                                /*std::cout << "isolated tri : new brokenEdge: localId " << i << ", " << m_brokenEdges.size() << " th edge, v1=" << e.nodeId[0]
                                    << ", v2=" << e.nodeId[1] << ", cPos=" << cutPos << std::endl;*/
                                m_brokenEdges.push_back(e);
                            }
                        }
                        else if(thisTriIsIsolated)//manually add this broken edge info to the triangle if nesscary
                        { 
                            if (DEBUG) std::cout << "manually add this broken edge info to the reset triangle...\n";
                            m_triangles[tidGlobal].numberCuts++;
                            m_triangles[tidGlobal].brokenEdgeId.push_back(checkIfEdgeBroken(e.nodeId[0], e.nodeId[1]));
                        }
                    }
                }
                if (DEBUG) std::cout << "isolated tri " << tidGlobal << " has been cut with local edge " << edgeBroken[0] << edgeBroken[1] << edgeBroken[2] <<", brokenType " << m_triangles[tidGlobal].brokenType<< std::endl;
                m_triangles[tidGlobal].brokenType = 3 - edgeBroken[1] - 2 * edgeBroken[2];
            }      
            else if (edgeBroken[0] + edgeBroken[1] + edgeBroken[2] <= 1)
            {
                //not handle the case where no more than one edge got intersected
                std::cout << "Warning : special case detected -- this triangle is not properly cut by the blade...\n";
                //m_triangles[tidGlobal].doneHandlingCut = true; //remove this tri?
            }
            //commented out for now, splitting triangle may cause un-connected cuts for pattern cutting                       
            /*if (startCutBaryCoords[0]>=0.25 && startCutBaryCoords[1] >= 0.25 && startCutBaryCoords[2] >= 0.25)
            {
                m_splitTriangle.push_back(MeshSplitTriangle((size_t)startCutTriIdGlobal, m_triangles[startCutTriIdGlobal].nodeId));
                m_splitTriangle[m_splitTriangle.size() - 1].baryCoords = startCutBaryCoords;
            }*/
        }
    }

    bool
    SurfaceCuttingManager::findToolSurfaceContactPair(std::shared_ptr<ToolState> info, std::tuple<size_t, Vec3d*, Vec3d>& tuple)
    {
        size_t vid = 0;
        Vec3d offset;//from vertex to tooltip
        std::set<size_t> candidateTris(info->triId);
        for (auto triId = 0; triId < m_trianglesVertices.size(); triId++)
        {
            candidateTris.insert(triId);
        }
        for (std::set<size_t>::iterator it = candidateTris.begin(); it != candidateTris.end(); it++)
        {
            Vec3f baryCoords;
            if(projectPointToTriangle(info->toolTipEndPos, m_trianglesVertices[*it], baryCoords) && abs(pointToPlaneDistance(info->toolTipEndPos, m_trianglesVertices[*it])) < 0.8)
            {
                int minVertLocalId = 0;
                if (baryCoords[1] > baryCoords[0] && baryCoords[1] > baryCoords[2])
                    minVertLocalId = 1;
                else if(baryCoords[2] > baryCoords[0] && baryCoords[2] > baryCoords[1])
                    minVertLocalId = 2;
                vid = m_trianglesVertices[*it].at(minVertLocalId);
                offset = info->toolTipEndPos - m_vertexPositions[vid];
                tuple = std::make_tuple(vid, &info->toolTipEndPos, 0.1*offset); 
                return true;
            }
        }
        return false;
    }


    int
    SurfaceCuttingManager::checkTriangleOnWhichSideOfPlane(TriangleArray &tri, const Vec3d &v1, const Vec3d &v2, const Vec3d &v3)
    {
        Vec3d triVert0 = tri[0] < m_vertexPositions.size() ? m_vertexPositions[tri[0]] : Vec3d(0, 0, 0);
        Vec3d triVert1 = tri[1] < m_vertexPositions.size() ? m_vertexPositions[tri[1]] : Vec3d(0, 0, 0);
        Vec3d triVert2 = tri[2] < m_vertexPositions.size() ? m_vertexPositions[tri[2]] : Vec3d(0, 0, 0);
        Vec3d triCenter = triVert0 / 3 + triVert1 / 3 + triVert2 / 3;
        Vec3d vec1 = v2 - v1;
        Vec3d vec2 = v3 - v1;
        Vec3d vec3 = triCenter - v1;
        return sgn(vec1.cross(vec2).dot(vec3));

    }

    double 
    SurfaceCuttingManager::computeVertexAngleInTriangle(TriangleArray & tri, size_t vid)
    {
        int localVid = findVertexInTriangle(vid, tri);
        Vec3d eA = m_vertexPositions[tri[(localVid + 1) % 3]] - m_vertexPositions[tri[localVid]];
        Vec3d eB= m_vertexPositions[tri[(localVid + 2) % 3]] - m_vertexPositions[tri[localVid]];
        return eA.dot(eB) / eA.norm() / eB.norm();
    }

}