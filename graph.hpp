#pragma once

#include "geom.hpp"

#include <tuple>

struct Edge {
    int from;
    int to;
    int weight;

    Edge(int from, int to, int w): from(from), to(to), weight(w) {}

    bool operator<(const Edge& other) const {
        return std::tie(weight, from, to) < std::tie(other.weight, other.from, other.to);
    }

    bool operator>(const Edge& other) const {
        return std::tie(weight, from, to) > std::tie(other.weight, other.from, other.to);
    }
};

class SqGrid
{
public:
    SqGrid(const Point& top_left, double dist, int axis_count)
            : tl_(top_left)
            , dist_(dist)
            , axis_count_(axis_count)
    {
        // grid_.resize(axis_count, std::vector<Point>(axis_count));
        grid_.resize(axis_count * axis_count);

        for (int i = 0; i < axis_count; i++) {
            for (int j = 0; j < axis_count; j++) {
                grid_[i * axis_count + j] = Point(top_left.x() + j * dist, top_left.y() + i * dist);
            }
        }
    }

    const Point* operator[](int i) const {
        return &grid_.data()[i];
    }

    int get_index(const Point& p) const {
        int x = p.x() - tl_.x();
        int y = p.y() - tl_.y();

        int i = y / dist_;
        int j = x / dist_;
        return i * axis_count_ + j;
    }

    const std::vector<Point>& vertices() const {
        return grid_;
    }

private:

    Point tl_;
    double dist_;
    int axis_count_;
    std::vector<Point> grid_;
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
        adj_.at(i).emplace_back(i, j, w);
    }

    void add_arc(const Edge& arc) {
        adj_.at(arc.from).push_back(arc);
    }

    void add_edge(const Edge& edge) {
        add_edge(edge.from, edge.to, edge.weight);
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

    bool intersects_with_segment(const Point& a, const Point& b) {
        boost::geometry::model::segment<Point> seg(a, b);

        for (auto& adj_list : adj_) {
            for (auto& e : adj_list) {
                boost::geometry::model::segment<Point> edge_seg(vertices_[e.from], vertices_[e.to]);

                std::vector<Point> pts;
                boost::geometry::intersection(seg, edge_seg, pts);

                if (!pts.empty()) {
                    return true;
                }
            }
        }

        return false;
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

    virtual std::vector<int> get_tour_to(int v);
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