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

    const std::vector<Point>& vertices() {
        return vertices_;
    }

    const std::vector<Edge>& adj(int i) {
        return adj_[i];
    }

    const std::vector<std::vector<Edge>>& adj() {
        return adj_;
    }

private:
    std::vector<Point> vertices_;
    std::vector<std::vector<Edge>> adj_;
};