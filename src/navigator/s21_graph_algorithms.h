#ifndef S21_GRAPH_ALGORITHMS_H
#define S21_GRAPH_ALGORITHMS_H

#include <algorithm>
#include <limits>
#include <cmath>
#include <random>


#include "s21_stack.h"
#include "s21_queue.h"
#include <set>
#include "s21_graph.h"

namespace s21 {
    class GraphAlgorithms {
    public:
        GraphAlgorithms() = default;
        ~GraphAlgorithms() = default;

        struct TsmResult {
            std::vector<int> vertices;
            double distance;
        };

        static std::vector<int> DepthFirstSearch(Graph &graph, int start_vertex);
        static std::vector<int> BreadthFirstSearch(Graph &graph, int start_vertex);
        static int GetShortestPathBetweenVertices(Graph &graph, int vertex1, int vertex2);
        static std::vector<std::vector<int>> GetShortestPathsBetweenAllVertices(Graph &graph);
        static std::vector<std::vector<int>> GetLeastSpanningTree(Graph &graph);
        static TsmResult SolveTravelingSalesmanProblem(Graph &graph);
    private:
        struct Ant;

        static int MinDistance(const std::vector<int> &distance, const std::vector<bool> &visited);


        struct DFSData {
            int vertex;
            bool visited;

            DFSData(int v, bool vis) : vertex(v), visited(vis) {}
        };

        struct BFSData {
            int vertex;
            int distance;

            BFSData() : vertex(0), distance(0) {}
            BFSData(int v, int d) : vertex(v), distance(d) {}
        };

        class AntColony {
        private:
            const Graph& graph_;
            int num_ants_;
            double alpha_;
            double beta_;
            double evaporation_rate_;
            double pheromone_deposit_weight_;
            std::vector<std::vector<double>> pheromones_;
            std::vector<std::vector<double>> visibility_;
            std::vector<int> best_tour_;
            double best_distance_;

            std::default_random_engine random_engine_;

        public:
            AntColony(const Graph& graph, int num_ants, double alpha, double beta,
                      double evaporation_rate, double pheromone_deposit_weight)
                    : graph_(graph),
                      num_ants_(num_ants),
                      alpha_(alpha),
                      beta_(beta),
                      evaporation_rate_(evaporation_rate),
                      pheromone_deposit_weight_(pheromone_deposit_weight),
                      random_engine_(std::random_device{}())
            {
                initializePheromones();
                initializeVisibility();
            }

            void solve(int num_iterations) {
                for (int i = 0; i < num_iterations; ++i) {
                    std::vector<std::vector<int>> ant_tours = generateAntTours();
                    updatePheromones(ant_tours);
                    updateBestTour(ant_tours);
                    evaporatePheromones();
                }
            }

            [[nodiscard]] TsmResult getBestTour() const {
                TsmResult result;
                result.vertices = best_tour_;
                result.distance = best_distance_;
                return result;
            }

        private:
            void initializePheromones() {
                int num_vertices = graph_.getNumberOfVertices();
                pheromones_.resize(num_vertices, std::vector<double>(num_vertices, 1.0));
            }

            void initializeVisibility() {
                int num_vertices = graph_.getNumberOfVertices();
                visibility_.resize(num_vertices, std::vector<double>(num_vertices, 0.0));

                for (int i = 0; i < num_vertices; ++i) {
                    for (int j = 0; j < num_vertices; ++j) {
                        if (i != j) {
                            visibility_[i][j] = 1.0 / graph_.GetWeight(i, j);
                        }
                    }
                }
            }

            std::vector<std::vector<int>> generateAntTours() {
                std::vector<std::vector<int>> ant_tours;

                for (int ant = 0; ant < num_ants_; ++ant) {
                    int start_vertex = 0;
                    std::vector<int> tour = generateAntTour(start_vertex);
                    ant_tours.push_back(tour);
                }

                return ant_tours;
            }

            std::vector<int> generateAntTour(int start_vertex) {
                int num_vertices = graph_.getNumberOfVertices();
                std::vector<int> tour;
                tour.push_back(start_vertex);

                std::vector<bool> visited(num_vertices, false);
                visited[start_vertex] = true;

                for (int step = 1; step < num_vertices; ++step) {
                    int current_vertex = tour.back();
                    int next_vertex = selectNextVertex(current_vertex, visited);
                    tour.push_back(next_vertex);
                    visited[next_vertex] = true;
                }

                return tour;
            }

            int selectNextVertex(int current_vertex, const std::vector<bool>& visited) {
                int num_vertices = graph_.getNumberOfVertices();
                double total_probability = 0.0;

                std::vector<double> probabilities(num_vertices, 0.0);

                for (int vertex = 0; vertex < num_vertices; ++vertex) {
                    if (!visited[vertex] && graph_.GetWeight(current_vertex, vertex) > 0.0) {
                        probabilities[vertex] = calculateTransitionProbability(current_vertex, vertex);
                        total_probability += probabilities[vertex];
                    }
                }

                std::random_device rd;
                std::mt19937 randomEngine(rd());
                std::uniform_real_distribution<double> distribution(0.0, total_probability);

                double random_value = distribution(randomEngine);

                double cumulative_probability = 0.0;
                for (int vertex = 0; vertex < num_vertices; ++vertex) {
                    if (!visited[vertex]) {
                        cumulative_probability += probabilities[vertex];
                        if (cumulative_probability >= random_value) {
                            return vertex;
                        }
                    }
                }

                return -1;
            }

            double calculateTransitionProbability(int current_vertex, int next_vertex) const {
                return std::pow(pheromones_[current_vertex][next_vertex], alpha_) *
                       std::pow(visibility_[current_vertex][next_vertex], beta_);
            }

            void updatePheromones(const std::vector<std::vector<int>>& ant_tours) {
                int num_vertices = graph_.getNumberOfVertices();

                for (int i = 0; i < num_vertices; ++i) {
                    for (int j = 0; j < num_vertices; ++j) {
                        pheromones_[i][j] *= (1.0 - evaporation_rate_);
                    }
                }

                for (const auto& tour : ant_tours) {
                    double tour_distance = calculateTourDistance(tour);

                    for (int i = 0; i < num_vertices - 1; ++i) {
                        int from_vertex = tour[i];
                        int to_vertex = tour[i + 1];

                        pheromones_[from_vertex][to_vertex] += pheromone_deposit_weight_ / tour_distance;
                        pheromones_[to_vertex][from_vertex] += pheromone_deposit_weight_ / tour_distance;
                    }
                }
            }

            double calculateTourDistance(const std::vector<int>& tour) const {
                double distance = 0.0;
                int num_vertices = graph_.getNumberOfVertices();

                for (int i = 0; i < num_vertices - 1; ++i) {
                    int from_vertex = tour[i];
                    int to_vertex = tour[i + 1];
                    distance += graph_.GetWeight(from_vertex, to_vertex);
                }

                int last_vertex = tour.back();
                int first_vertex = tour.front();
                distance += graph_.GetWeight(last_vertex, first_vertex);

                return distance;
            }

            void updateBestTour(const std::vector<std::vector<int>>& ant_tours) {
                for (const auto& tour : ant_tours) {
                    double tour_distance = calculateTourDistance(tour);
                    if (best_tour_.empty() || tour_distance < best_distance_) {
                        best_tour_ = tour;
                        best_distance_ = tour_distance;
                    }
                }
            }

            void evaporatePheromones() {
                int num_vertices = graph_.getNumberOfVertices();

                for (int i = 0; i < num_vertices; ++i) {
                    for (int j = 0; j < num_vertices; ++j) {
                        pheromones_[i][j] *= (1.0 - evaporation_rate_);
                    }
                }
            }

            int randomEngine() {
                return random_engine_();
            }
        };


    };
}
#endif // S21_GRAPH_ALGORITHMS_H
