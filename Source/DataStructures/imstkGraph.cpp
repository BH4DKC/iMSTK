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

#include <algorithm>
#include <numeric>
#include <map>
#include <climits>
#undef min
#undef max

#include "imstkGraph.h"
#include "imstkParallelUtils.h"

#include <g3log/g3log.hpp>

namespace imstk
{
void
Graph::addEdge(const size_t v, const size_t w)
{
    if (v < m_adjList.size() && w < m_adjList.size())
    {
        m_adjList[v].insert(w);
        m_adjList[w].insert(v);
    }
    else
    {
        LOG(WARNING) << "Vertex id exceeds the graph size: cannot add edge!" << std::endl;
    }
}

void
Graph::print() const
{
    std::cout << "Graph: " << "\nTotal nodes: " << m_adjList.size() << "\nAdjacency:" << std::endl;

    for (size_t i = 0; i < m_adjList.size(); i++)
    {
        std::cout << "\t[" << i << "] : ";

        for (auto v : m_adjList[i])
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }
}

std::pair<std::vector<unsigned short>, unsigned short>
Graph::doColoring(ColoringMethod method, bool print /*= false*/) const
{
    return method == ColoringMethod::WelshPowell ?
           doColoringWelshPowell(print) :
           doColoringGreedy(print);
}

std::pair<std::vector<unsigned short>, unsigned short>
Graph::doColoringWelshPowell(bool print /*= false*/) const
{
    const auto numNodes = m_adjList.size();

    // Must initialize colors to inf number
    std::vector<unsigned short> colors(numNodes, std::numeric_limits<unsigned short>::max());

    // Count the number of neighbors for each node
    std::vector<size_t> neighborCounts(numNodes);
    ParallelUtils::parallelFor(numNodes,
        [&](const size_t idx)
        {
            neighborCounts[idx] = m_adjList[idx].size();
        });

    std::vector<unsigned short> coloringOrder(numNodes);
    std::iota(coloringOrder.begin(), coloringOrder.end(), static_cast<unsigned short>(0));

    std::vector<bool> coloredNodes;
    unsigned short    color = 0;
    while (coloringOrder.size() > 0)
    {
        coloredNodes.resize(coloringOrder.size());
        coloredNodes.assign(coloringOrder.size(), false);
        coloredNodes.front() = true;

        // Node with largest number of neighbors is processed first
        tbb::parallel_sort(coloringOrder.begin(), coloringOrder.end(),
            [&](const size_t idx0, const size_t idx1) {
                return neighborCounts[idx0] > neighborCounts[idx1];
            });
        colors[coloringOrder.front()] = color; // The first node is colorized

        // Cannot run in parallel
        for (size_t i = 1; i < coloringOrder.size(); ++i)
        {
            const auto u   = coloringOrder[i];
            bool       bOK = true;
            for (const auto v : m_adjList[u])
            {
                // Check if any neighbor node has the same color as the first processing node
                if (colors[v] == color)
                {
                    bOK = false;
                    break;
                }
            }
            if (bOK)
            {
                colors[u]       = color;
                coloredNodes[i] = true;
            }
        }

        // Done with the current color
        ++color;

        // Remove colorized nodes
        size_t writeIdx = 0;
        for (size_t readIdx = 1; readIdx < coloringOrder.size(); ++readIdx)
        {
            if (!coloredNodes[readIdx])
            {
                coloringOrder[writeIdx++] = coloringOrder[readIdx];
            }
        }
        coloringOrder.resize(writeIdx);
    }

    // print the result
    if (print)
    {
        std::map<size_t, size_t> verticesPerColor;
        std::cout << "Num. of nodes: " << numNodes << " | Num. of colors: " << color << std::endl;
        for (size_t i = 0; i < numNodes; ++i)
        {
            std::cout << "V " << i << "-C " << colors[i] << " | ";
            verticesPerColor[colors[i]]++;
        }
        std::cout << std::endl;
        std::cout << "Vertices per color: " << std::endl;
        for (const auto& kv : verticesPerColor)
        {
            std::cout << "C: " << kv.first << " - " << kv.second << std::endl;
        }
        std::cout << std::endl;
    }

    return std::make_pair(colors, color);
}

std::pair<std::vector<unsigned short>, unsigned short>
Graph::doColoringGreedy(bool print /*= false*/) const
{
    const auto                  numNodes = m_adjList.size();
    std::vector<unsigned short> colors(numNodes, std::numeric_limits<unsigned short>::max());
    std::vector<bool>           available(numNodes, false);

    colors[0] = 0;
    unsigned short numColors = 0;

    // Assign colors to remaining V-1 vertices
    for (size_t u = 1; u < numNodes; ++u)
    {
        // Process all adjacent vertices and flag their colors
        // as unavailable
        for (const auto& i : m_adjList[u])
        {
            if (colors[i] != std::numeric_limits<unsigned short>::max())
            {
                available[colors[i]] = true;
            }
        }

        // Find the first available color
        unsigned short cr;
        for (cr = 0; cr < numNodes; cr++)
        {
            if (!available[cr])
            {
                break;
            }
        }
        colors[u] = cr; // Assign the found color
        if (cr + 1 > numColors)
        {
            numColors = cr + 1;
        }

        // Reset the values back to false for the next iteration
        for (const auto& i : m_adjList[u])
        {
            if (colors[i] != std::numeric_limits<unsigned short>::max())
            {
                available[colors[i]] = false;
            }
        }
    }

    // print the result
    if (print)
    {
        std::map<size_t, size_t> verticesPerColor;
        std::cout << "Num. of nodes: " << numNodes << " | Num. of colors: " << numColors << std::endl;
        for (size_t i = 0; i < numNodes; ++i)
        {
            std::cout << "V " << i << "-C " << colors[i] << " | ";
            verticesPerColor[colors[i]]++;
        }
        std::cout << std::endl;
        std::cout << "Vertices per color: " << std::endl;
        for (const auto& kv : verticesPerColor)
        {
            std::cout << "C: " << kv.first << " - " << kv.second << std::endl;
        }
        std::cout << std::endl;
    }

    return std::make_pair(colors, numColors);
}
}
