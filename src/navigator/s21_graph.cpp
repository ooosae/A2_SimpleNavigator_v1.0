#include "s21_graph.h"

namespace s21 {
    void Graph::LoadGraphFromFile(const std::string &filename) {
        std::ifstream inputFile(filename);

        if (!inputFile.is_open()) {
            throw std::runtime_error("Error: Unable to open file " + filename);
        }

        adjacencyMatrix.clear();
        int size;
        inputFile >> size;

        adjacencyMatrix.resize(size, std::vector<int>(size, 0));

        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                inputFile >> adjacencyMatrix[i][j];
            }
        }

        inputFile.close();
    }


    void Graph::ExportGraphToDot(const std::string &filename) {
        std::ofstream outputFile(filename);

        if (!outputFile.is_open()) {
            throw std::runtime_error("Error: Unable to open file " + filename);
        }

        outputFile << GetDotHeader();

        for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
            outputFile << "  " << GetDotNodeLabel(i + 1) << std::endl;
            for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
                if (adjacencyMatrix[i][j] != 0) {
                    outputFile << "  " << GetDotEdge(i + 1, j + 1) << std::endl;
                }
            }
        }

        outputFile << "}" << std::endl;
        outputFile.close();
    }

    std::string Graph::GetDotHeader() {
        return "digraph G {\n";
    }

    std::string Graph::GetDotNodeLabel(int vertex) {
        return "  " + std::to_string(vertex) + " [label=\"" + std::to_string(vertex) + "\"]";
    }

    std::string Graph::GetDotEdge(int from, int to) {
        return "  " + std::to_string(from) + " -> " + std::to_string(to);
    }

    std::vector<int> Graph::getNeighbors(int vertex) const {
        std::vector<int> neighbors;

        if (vertex < 0 || vertex >= static_cast<int>(adjacencyMatrix.size())) {
            throw std::out_of_range("Error: Invalid vertex index.");
        }

        for (size_t i = 0; i < adjacencyMatrix[vertex].size(); ++i) {
            if (adjacencyMatrix[vertex][i] != 0) {
                neighbors.push_back(i);
            }
        }

        return neighbors;
    }

    int Graph::getNumberOfVertices() const {
        return static_cast<int>(adjacencyMatrix.size());
    }

    int Graph::GetWeight(int vertex1, int vertex2) const {
        int numVertices = getNumberOfVertices();
        if (vertex1 < 0 || vertex1 >= numVertices || vertex2 < 0 || vertex2 >= numVertices) {
            throw std::out_of_range("Error: Invalid vertex index.");
        }
        return adjacencyMatrix[vertex1][vertex2];
    }

    size_t Graph::GetSize() {
        return adjacencyMatrix.size();
    }
}

