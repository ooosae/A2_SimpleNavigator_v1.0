#include <queue>
#include "s21_graph_algorithms.h"

namespace s21 {

    std::vector<int> GraphAlgorithms::DepthFirstSearch(Graph &graph, int start_vertex) {
        std::vector<int> result;

        int numVertices = graph.getNumberOfVertices();

        if (start_vertex < 1 || start_vertex > numVertices) {
            throw std::out_of_range("Error: Invalid start vertex index.");
        }

        std::vector<bool> visited(numVertices, false);
        stack<int> dfsStack;

        dfsStack.push(start_vertex - 1);
        visited[start_vertex - 1] = true;

        while (!dfsStack.empty()) {
            int currentVertex = dfsStack.top();
            dfsStack.pop();

            result.push_back(currentVertex + 1);

            for (int neighbor : graph.getNeighbors(currentVertex)) {
                if (!visited[neighbor]) {
                    dfsStack.push(neighbor);
                    visited[neighbor] = true;
                }
            }
        }

        return result;
    }


    std::vector<int> GraphAlgorithms::BreadthFirstSearch(Graph &graph, int start_vertex) {
        std::vector<int> result;

        int numVertices = graph.getNumberOfVertices();

        if (start_vertex < 1 || start_vertex > numVertices) {
            throw std::out_of_range("Error: Invalid start vertex index.");
        }

        std::vector<bool> visited(numVertices, false);

        queue<BFSData> bfsQueue;
        bfsQueue.push(BFSData(start_vertex - 1, 0));
        visited[start_vertex - 1] = true;

        while (!bfsQueue.empty()) {
            BFSData current = bfsQueue.front();
            bfsQueue.pop();

            result.push_back(current.vertex + 1);

            for (int neighbor : graph.getNeighbors(current.vertex)) {
                if (!visited[neighbor]) {
                    bfsQueue.push(BFSData(neighbor, current.distance + 1));
                    visited[neighbor] = true;
                }
            }
        }
        return result;
    }

    int GraphAlgorithms::GetShortestPathBetweenVertices(Graph &graph, int vertex1, int vertex2) {
        int numVertices = graph.getNumberOfVertices();

        if (vertex1 < 1 || vertex1 > numVertices || vertex2 < 1 || vertex2 > numVertices) {
            throw std::out_of_range("Error: Invalid vertex indices.");
        }

        std::vector<int> distance(numVertices, std::numeric_limits<int>::max());
        distance[vertex1 - 1] = 0;

        std::vector<bool> visited(numVertices, false);

        for (int count = 0; count < numVertices - 1; ++count) {
            int u = MinDistance(distance, visited);
            visited[u] = true;

            for (int v = 0; v < numVertices; ++v) {
                if (!visited[v] && graph.GetWeight(u, v) != 0 &&
                    distance[u] != std::numeric_limits<int>::max() &&
                    distance[u] + graph.GetWeight(u, v) < distance[v]) {
                    distance[v] = distance[u] + graph.GetWeight(u, v);
                }
            }
        }

        return distance[vertex2 - 1];
    }

    std::vector<std::vector<int>> GraphAlgorithms::GetShortestPathsBetweenAllVertices(Graph &graph) {
        int numVertices = graph.getNumberOfVertices();
        std::vector<std::vector<int>> shortestPaths(numVertices, std::vector<int>(numVertices, std::numeric_limits<int>::max()));

        for (int i = 0; i < numVertices; ++i) {
            for (int j = 0; j < numVertices; ++j) {
                if (i == j) {
                    shortestPaths[i][j] = 0;
                } else {
                    int weight = graph.GetWeight(i, j);
                    if (weight != 0) {
                        shortestPaths[i][j] = weight;
                    }
                }
            }
        }

        for (int k = 0; k < numVertices; ++k) {
            for (int i = 0; i < numVertices; ++i) {
                for (int j = 0; j < numVertices; ++j) {
                    if (shortestPaths[i][k] != std::numeric_limits<int>::max() &&
                        shortestPaths[k][j] != std::numeric_limits<int>::max() &&
                        shortestPaths[i][k] + shortestPaths[k][j] < shortestPaths[i][j]) {
                        shortestPaths[i][j] = shortestPaths[i][k] + shortestPaths[k][j];
                    }
                }
            }
        }

        return shortestPaths;
    }

    int GraphAlgorithms::MinDistance(const std::vector<int> &distance, const std::vector<bool> &visited) {
        int minDistance = std::numeric_limits<int>::max();
        int minIndex = -1;

        for (size_t v = 0; v < distance.size(); ++v) {
            if (!visited[v] && distance[v] <= minDistance) {
                minDistance = distance[v];
                minIndex = v;
            }
        }

        return minIndex;
    }

    std::vector<std::vector<int>> GraphAlgorithms::GetLeastSpanningTree(Graph& graph) {
        int numVertices = graph.getNumberOfVertices();

        std::vector<std::vector<int>> minSpanningTree(numVertices, std::vector<int>(numVertices, 0));

        std::priority_queue<std::pair<int, std::pair<int, int>>, std::vector<std::pair<int, std::pair<int, int>>>, std::greater<>> pq;

        std::vector<bool> visited(numVertices, false);

        int startVertex = 0;
        visited[startVertex] = true;

        for (int neighbor : graph.getNeighbors(startVertex)) {
            pq.push({ graph.GetWeight(startVertex, neighbor), {startVertex, neighbor} });
        }

        while (!pq.empty()) {
            auto edge = pq.top();
            pq.pop();

            int weight = edge.first;
            int from = edge.second.first;
            int to = edge.second.second;

            if (!visited[to]) {
                visited[to] = true;
                minSpanningTree[from][to] = weight;
                minSpanningTree[to][from] = weight;

                for (int neighbor : graph.getNeighbors(to)) {
                    pq.push({ graph.GetWeight(to, neighbor), {to, neighbor} });
                }
            }
        }

        return minSpanningTree;
    }

    GraphAlgorithms::TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(Graph& graph) {
        // Параметры алгоритма муравьиной колонии
        int num_ants = 10;
        double alpha = 1.0;
        double beta = 2.0;
        double evaporation_rate = 0.5;
        double pheromone_deposit_weight = 100.0;

        // Количество итераций алгоритма
        int num_iterations = 100;

        // Создание объекта AntColony и решение задачи коммивояжера
        AntColony ant_colony(graph, num_ants, alpha, beta, evaporation_rate, pheromone_deposit_weight);
        ant_colony.solve(num_iterations);

        // Получение и возвращение лучшего пути и его расстояния
        return ant_colony.getBestTour();
    }

}


