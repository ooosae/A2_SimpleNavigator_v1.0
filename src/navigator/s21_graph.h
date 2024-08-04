#ifndef NAVIGATOR_S21_GRAPH_H
#define NAVIGATOR_S21_GRAPH_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <limits>

namespace s21 {
    class Graph {
    public:
        Graph() = default;
        ~Graph() = default;

        size_t GetSize();

        void LoadGraphFromFile(const std::string &filename);
        void ExportGraphToDot(const std::string &filename);

        [[nodiscard]] std::vector<int> getNeighbors(int vertex) const;
        [[nodiscard]] int getNumberOfVertices() const;
        [[nodiscard]] int GetWeight(int vertex1, int vertex2) const;

    private:
        std::vector<std::vector<int>> adjacencyMatrix;

        static std::string GetDotHeader();
        static std::string GetDotNodeLabel(int vertex);
        static std::string GetDotEdge(int from, int to);
    };
}

#endif //NAVIGATOR_S21_GRAPH_H
