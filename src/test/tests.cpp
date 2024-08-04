#include <gtest/gtest.h>
#include "s21_graph.h"
#include "s21_graph_algorithms.h"
#include "unordered_set"

bool containsAllNumbers(const std::vector<int>& vec) {
    if (vec.empty()) {
        return false;
    }

    std::unordered_set<int> uniqueNumbers(vec.begin(), vec.end());

    for (size_t i = 0; i < vec.size(); ++i) {
        if (uniqueNumbers.find(i) == uniqueNumbers.end()) {
            return false;
        }
    }

    return true;
}

TEST(GraphAlgorithmsTest, BreadthFirstSearch1) {
    s21::Graph graph;
    graph.LoadGraphFromFile("./data/graph2.txt");

    std::vector<int> bfsResult1 = s21::GraphAlgorithms::BreadthFirstSearch(graph, 1);
    EXPECT_EQ(std::vector<int>({1, 2, 3, 4}), bfsResult1);

    std::vector<int> bfsResult2 = s21::GraphAlgorithms::BreadthFirstSearch(graph, 3);
    EXPECT_EQ(std::vector<int>({3, 4}), bfsResult2);
}

TEST(GraphAlgorithmsTest, DepthFirstSearch1) {
    s21::Graph graph;
    graph.LoadGraphFromFile("./data/graph2.txt");

    std::vector<int> dfsResult1 = s21::GraphAlgorithms::DepthFirstSearch(graph, 1);
    EXPECT_EQ(std::vector<int>({1, 3, 4, 2}), dfsResult1);

    std::vector<int> dfsResult2 = s21::GraphAlgorithms::DepthFirstSearch(graph, 3);
    EXPECT_EQ(std::vector<int>({3, 4}), dfsResult2);
}

TEST(GraphAlgorithmsTest, BreadthFirstSearch2) {
    s21::Graph graph;
    graph.LoadGraphFromFile("./data/graph1.txt");

    std::vector<int> bfsResult1 = s21::GraphAlgorithms::BreadthFirstSearch(graph, 1);
    EXPECT_EQ(std::vector<int>({1, 4, 2, 3, 5}), bfsResult1);

    std::vector<int> bfsResult2 = s21::GraphAlgorithms::BreadthFirstSearch(graph, 2);
    EXPECT_EQ(std::vector<int>({2, 4, 5, 3, 1}), bfsResult2);

    std::vector<int> bfsResult3 = s21::GraphAlgorithms::BreadthFirstSearch(graph, 3);
    EXPECT_EQ(std::vector<int>({3, 1, 4, 2, 5}), bfsResult3);
}

TEST(GraphAlgorithmsTest, DepthFirstSearch2) {
    s21::Graph graph;
    graph.LoadGraphFromFile("./data/graph1.txt");

    std::vector<int> dfsResult1 = s21::GraphAlgorithms::DepthFirstSearch(graph, 1);
    EXPECT_EQ(std::vector<int>({1, 4, 3, 2, 5}), dfsResult1);

    std::vector<int> dfsResult2 = s21::GraphAlgorithms::DepthFirstSearch(graph, 2);
    EXPECT_EQ(std::vector<int>({2, 5, 3, 1, 4}), dfsResult2);

    std::vector<int> dfsResult3 = s21::GraphAlgorithms::DepthFirstSearch(graph, 3);
    EXPECT_EQ(std::vector<int>({3, 1, 4, 2, 5}), dfsResult3);
}

TEST(GraphAlgorithmsTest, GetShortestPathBetweenVertices) {
    s21::Graph graph1;
    graph1.LoadGraphFromFile("./data/graph2.txt");

    int shortestPath1 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph1, 1, 4);
    EXPECT_EQ(2, shortestPath1);

    int shortestPath2 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph1, 2, 4);
    EXPECT_EQ(2, shortestPath2);

    int shortestPath3 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph1, 3, 1);
    EXPECT_EQ(std::numeric_limits<int>::max(), shortestPath3);

    s21::Graph graph2;
    graph2.LoadGraphFromFile("./data/graph1.txt");

    int shortestPath4 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph2, 1, 4);
    EXPECT_EQ(1, shortestPath4);

    int shortestPath5 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph2, 2, 3);
    EXPECT_EQ(2, shortestPath5);

    int shortestPath6 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph2, 5, 3);
    EXPECT_EQ(1, shortestPath6);

    s21::Graph graph3;
    graph2.LoadGraphFromFile("./data/graph3.txt");

    int shortestPath7 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph2, 3, 4);
    EXPECT_EQ(15, shortestPath7);

    int shortestPath8 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph2, 3, 2);
    EXPECT_EQ(7, shortestPath8);

    int shortestPath9 = s21::GraphAlgorithms::GetShortestPathBetweenVertices(graph2, 2, 6);
    EXPECT_EQ(std::numeric_limits<int>::max(), shortestPath9);
}

TEST(GraphAlgorithmsTest, GetShortestPathsBetweenAllVertices) {
    s21::Graph graph;
    graph.LoadGraphFromFile("./data/graph3.txt");

    std::vector<std::vector<int>> shortestPaths1 = s21::GraphAlgorithms::GetShortestPathsBetweenAllVertices(graph);
    EXPECT_EQ(6, shortestPaths1.size());
    EXPECT_EQ(0, shortestPaths1[0][0]);
    EXPECT_EQ(19, shortestPaths1[0][1]);
    EXPECT_EQ(22, shortestPaths1[0][2]);
    EXPECT_EQ(27, shortestPaths1[0][3]);
    EXPECT_EQ(77, shortestPaths1[0][4]);
    EXPECT_EQ(std::numeric_limits<int>::max(), shortestPaths1[0][5]);
}

TEST(GraphAlgorithmsTest, GetLeastSpanningTree) {
    s21::Graph graph;
    graph.LoadGraphFromFile("./data/graph4.txt");

    std::vector<std::vector<int>> LST = s21::GraphAlgorithms::GetLeastSpanningTree(graph);
    

    EXPECT_EQ(LST[0][1], 1);
    EXPECT_EQ(LST[0][3], 1);
    EXPECT_EQ(LST[0][1], 1);
    EXPECT_EQ(LST[1][2], 1);
    EXPECT_EQ(LST[2][3], 0);
    EXPECT_EQ(LST[2][4], 0);

    EXPECT_EQ(LST[0][2], 0);
    EXPECT_EQ(LST[1][3], 0);
    EXPECT_EQ(LST[3][4], 0);
}

TEST(GraphAlgorithmsTest, GetLeastSpanningTree2) {
    s21::Graph graph6;
    graph6.LoadGraphFromFile("./data/graph6.txt");

    std::vector<std::vector<int>> minSpanningTree = s21::GraphAlgorithms::GetLeastSpanningTree(graph6);

    EXPECT_EQ(2, minSpanningTree[0][1]);
    EXPECT_EQ(2, minSpanningTree[1][0]);
    EXPECT_EQ(0, minSpanningTree[0][3]);
    EXPECT_EQ(4, minSpanningTree[4][6]);
    EXPECT_EQ(4, minSpanningTree[6][4]);
}


TEST(GraphAlgorithmsTest, GetLeastSpanningTree3) {
    s21::Graph graph8;
    graph8.LoadGraphFromFile("./data/graph8.txt");

    std::vector<std::vector<int>> minSpanningTree = s21::GraphAlgorithms::GetLeastSpanningTree(graph8);

    EXPECT_EQ(2, minSpanningTree[0][3]);
    EXPECT_EQ(2, minSpanningTree[3][0]);
    EXPECT_EQ(0, minSpanningTree[0][5]);
    EXPECT_EQ(1, minSpanningTree[2][4]);
    EXPECT_EQ(1, minSpanningTree[4][2]);
}

TEST(GraphAlgorithmsTest, TSP1) {
    s21::Graph graph8;
    graph8.LoadGraphFromFile("./data/graph1.txt");

    s21::GraphAlgorithms::TsmResult result = s21::GraphAlgorithms::SolveTravelingSalesmanProblem(graph8);

    EXPECT_EQ(0, result.vertices[0]);
    EXPECT_EQ(true, containsAllNumbers(result.vertices));
}

TEST(GraphAlgorithmsTest, TSP2) {
    s21::Graph graph8;
    graph8.LoadGraphFromFile("./data/graph9.txt");

    s21::GraphAlgorithms::TsmResult result = s21::GraphAlgorithms::SolveTravelingSalesmanProblem(graph8);

    EXPECT_EQ(0, result.vertices[0]);
    EXPECT_EQ(true, containsAllNumbers(result.vertices));
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
