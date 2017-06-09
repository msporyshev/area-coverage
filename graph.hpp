#pragma once

#include "geom.hpp"

class Graph
{
public:
    Graph() {}
    Graph(const std::vector<Point>& vertices): vertices_(vertices) {}

    void set_vertices(const std::vector<Point>& vertices) {
        vertices_ = vertices;
    }

    void add_v(const Point& p) {
        vertices_.push_back(p);
    }

    void add_arc(int i, int j) {
        adj_.at(i).push_back(j);
    }

    void add_edge(int i, int j) {
        add_arc(i, j);
        add_arc(j, i);
    }

    const std::vector<Point>& vertices() {
        return vertices_;
    }

    const std::vector<std::vector<int>>& adj() {
        return adj_;
    }

private:
    std::vector<Point> vertices_;
    std::vector<std::vector<int>> adj_;
};