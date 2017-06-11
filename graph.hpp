#pragma once

#include "geom.hpp"

struct Edge {
    int v;
    int weight;

    Edge(int v, int w): v(v), weight(w) {}
};

class Graph
{
public:
    Graph() {}
    Graph(const std::vector<Point>& vertices): vertices_(vertices), adj_(vertices.size()) {}

    void set_vertices(const std::vector<Point>& vertices) {
        vertices_ = vertices;
        adj_.resize(vertices_.size());
    }

    void add_v(const Point& p) {
        vertices_.push_back(p);
    }

    void add_arc(int i, int j, int w) {
        adj_.at(i).emplace_back(j, w);
    }

    void add_edge(int i, int j, int w) {
        add_arc(i, j, w);
        add_arc(j, i, w);
    }

    const std::vector<Point>& vertices() const {
        return vertices_;
    }

    const std::vector<Edge>& adj(int i) const {
        return adj_[i];
    }

    const std::vector<std::vector<Edge> >& adj() const {
        return adj_;
    }


    std::vector<Point> get_2d_tour(const std::vector<int>& ids) {
        std::vector<Point> res(ids.size());
        for (int i = 0; i < res.size(); i++) {
            res[i] = vertices_[ids[i]];
        }

        return res;
    }

private:
    std::vector<Point> vertices_;
    std::vector<std::vector<Edge>> adj_;
};

class GraphSearcher
{
public:
    GraphSearcher(const Graph& graph): graph_(graph) {}

protected:
    Graph graph_;
};

class StartVertexSearcher: public GraphSearcher
{
public:
    StartVertexSearcher(const Graph& graph, int start_vertex)
            : GraphSearcher(graph)
            , sv_(start_vertex)
            , prev_(graph.vertices().size(), -1)
            , distances_(graph.vertices().size(), -1.0)
    {}

    virtual const std::vector<double>& calc_distances() = 0;

    const std::vector<double>& get_distances() { return distances_; }

    virtual std::vector<Point> get_tour_to(int v);
protected:
    int sv_;
    std::vector<int> prev_;
    std::vector<double> distances_;
};

class BfsSearcher: public StartVertexSearcher
{
public:
    BfsSearcher(const Graph& graph, int start_vertex): StartVertexSearcher(graph, start_vertex) {}

    const std::vector<double>& calc_distances() override;
};