#include <iostream>

#include <ctime>
#include <chrono>

#include "s21_graph.h"
#include "s21_graph_algorithms.h"

void printMenu();
void findBreadthFirst(s21::Graph &graph);
void findDepthFirst(s21::Graph &graph);
void findShortestPath(s21::Graph &graph);
void findAllShortestPaths(s21::Graph &graph);
void findMinimumSpanningTree(s21::Graph &graph);
void solveTSP(s21::Graph &graph);
void importGraph(s21::Graph &graph);
void exportGraph(s21::Graph &graph);
void CompareTspAlgorithmsPerformance(s21::Graph &graph);

int main() {
    s21::Graph graph;
    printMenu();

    while (true) {
        int choice;

        std::cout << "Enter the operation number: ";
        std::cin >> choice;

        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input. Please enter a number." << std::endl;
            continue;
        }

        switch (choice) {
            case 1: {
                try {
                    importGraph(graph);
                }
                catch (std::exception &ex) {
                    std::cout << ex.what() << std::endl;
                }
                break;
            }
            case 2: {
                try {
                    findBreadthFirst(graph);
                }
                catch (std::exception &ex) {
                    std::cout << ex.what() << std::endl;

                }
                break;
            }
            case 3: {
                try {
                    findDepthFirst(graph);
                }
                catch (std::exception &ex) {
                    std::cout << ex.what() << std::endl;
                }
                break;
                case 4: {
                    try {
                        findShortestPath(graph);
                    }
                    catch (std::exception &ex) {
                        std::cout << ex.what() << std::endl;
                    }
                    break;
                }
                case 5: {
                    try {
                        findAllShortestPaths(graph);
                    }
                    catch (std::exception &ex) {
                        std::cout << ex.what() << std::endl;
                    }
                    break;
                }
                case 6: {
                    try {
                        findMinimumSpanningTree(graph);
                    }
                    catch (std::exception &ex) {
                        std::cout << ex.what() << std::endl;
                    }
                    break;
                }
                case 7: {
                    try {
                        solveTSP(graph);
                    }
                    catch (std::exception &ex) {
                        std::cout << ex.what() << std::endl;
                    }
                    break;
                }
                case 8: {
                    try {
                        exportGraph(graph);
                    }
                    catch (std::exception &ex) {
                        std::cout << ex.what() << std::endl;
                    }
                    break;
                }
                case 9: {
                    try {
                        CompareTspAlgorithmsPerformance(graph);
                    }
                    catch (std::exception &ex) {
                        std::cout << ex.what() << std::endl;
                    }
                    break;
                }
                case 10:
                    printMenu();
                    break;
                case 0:
                    return 0;
                default:
                    std::cout << "Invalid operation number. Please try again." << std::endl;
                    break;
            }
        }
    }
}

void printMenu() {
    std::cout << "============================" << std::endl;
    std::cout << "1. Load graph from file" << std::endl;
    std::cout << "2. Breadth-first traversal" << std::endl;
    std::cout << "3. Depth-first traversal" << std::endl;
    std::cout << "4. Find shortest path between vertices" << std::endl;
    std::cout << "5. Find shortest paths between all vertices" << std::endl;
    std::cout << "6. Find minimum spanning tree" << std::endl;
    std::cout << "7. Solve the Traveling Salesman Problem" << std::endl;
    std::cout << "8. Export graph to file" << std::endl;
    std::cout << "9. Check TSM ACO time" << std::endl;
    std::cout << "10. Print menu" << std::endl;
    std::cout << "0. Exit" << std::endl;
    std::cout << "============================" << std::endl;
}

void importGraph(s21::Graph &graph) {
    std::string filename;
    std::cout << "Enter the filename: ";
    std::cin >> filename;
    graph.LoadGraphFromFile(filename);
    std::cout << "Graph loaded from file." << std::endl;
}

void findBreadthFirst(s21::Graph &graph) {
    if (!graph.GetSize())
        throw std::runtime_error("Graph is empty");
    std::cout << "Enter the starting vertex: ";
    int startPoint;
    std::cin >> startPoint;
    std::vector<int> alg = s21::GraphAlgorithms::BreadthFirstSearch(graph, startPoint);
    std::cout << "Traversed vertices: ";
    for (auto i: alg) {
        std::cout << i << ' ';
    }
    std::cout << std::endl;
}

void findDepthFirst(s21::Graph &graph) {
    if (!graph.GetSize())
        throw std::runtime_error("Graph is empty");
    std::cout << "Enter the starting vertex: ";
    int startPoint;
    std::cin >> startPoint;
    std::vector<int> alg = s21::GraphAlgorithms::DepthFirstSearch(graph, startPoint);
    std::cout << "Traversed vertices: ";
    for (auto i: alg) {
        std::cout << i << ' ';
    }
    std::cout << std::endl;
}

void findShortestPath(s21::Graph &graph) {
    if (!graph.GetSize())
        throw std::runtime_error("Graph is empty");
    int vertex1, vertex2;
    std::cout << "Enter vertices: ";
    std::cin >> vertex1 >> vertex2;
    int length = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph, vertex1, vertex2);
    std::cout << "The length of the shortest path: " << length << std::endl;
}

void findAllShortestPaths(s21::Graph &graph) {
    if (!graph.GetSize())
        throw std::runtime_error("Graph is empty");
    std::vector<std::vector<int>> shortestPathsMatrix = s21::GraphAlgorithms::GetShortestPathsBetweenAllVertices(
            graph);
    std::cout << "All paths length:" << std::endl;
    for (const auto &row: shortestPathsMatrix) {
        for (int value: row) {
            if (value == std::numeric_limits<int>::max()) {
                std::cout << "INF ";
            } else {
                std::cout << value << " ";
            }
        }
        std::cout << std::endl;
    }
}

void findMinimumSpanningTree(s21::Graph &graph) {
    if (!graph.GetSize())
        throw std::runtime_error("Graph is empty");
    std::cout << "Minimum spanning tree:" << std::endl;
    std::vector<std::vector<int>> spanningTree = s21::GraphAlgorithms::GetLeastSpanningTree(graph);
    for (const auto &row: spanningTree) {
        for (int weight: row) {
            std::cout << weight << " ";
        }
        std::cout << std::endl;
    }
}

void solveTSP(s21::Graph &graph) {
    if (!graph.GetSize())
        throw std::runtime_error("Graph is empty");

    std::cout << "Ant algorithm" << std::endl;
    s21::GraphAlgorithms::TsmResult result = s21::GraphAlgorithms::SolveTravelingSalesmanProblem(
            graph);
    std::cout << "Optimal Tour: ";
    for (int vertex : result.vertices) {
        std::cout << vertex + 1 << " ";
    }
    std::cout << 1 << std::endl;
    std::cout << "Total Distance: " << result.distance << std::endl;
}

void exportGraph(s21::Graph &graph) {
    if (!graph.GetSize())
        throw std::runtime_error("Graph is empty");
    std::string filename;
    std::cout << "Enter the filename for export: ";
    std::cin >> filename;
    graph.ExportGraphToDot(filename);
    std::cout << "Graph exported to file." << std::endl;
}

void CompareTspAlgorithmsPerformance(s21::Graph &graph) {
    if (!graph.GetSize())
        throw std::runtime_error("Graph is empty");

    int iterations;
    std::cout << "Enter the number of iterations: ";
    std::cin >> iterations;

    auto startAnt = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        s21::GraphAlgorithms::SolveTravelingSalesmanProblem(graph);
    }
    auto endAnt = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedAnt = endAnt - startAnt;
    std::cout << "Ant algorithm time: " << elapsedAnt.count() << " seconds" << std::endl;
}
