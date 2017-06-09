#pragma once

#include "geom.hpp"
#include "graph.hpp"
#include "vis.hpp"

#include <iostream>
#include <queue>
#include <cmath>

namespace cgal = CGAL;

const int VIS_RADIUS = 30;

#define SQR(x) ((x)*(x))

class Planner
{
public:
    Planner(const Polygon& domain): domain_(domain) {}

    virtual std::vector<Point> calc_tour() = 0;
    virtual void vis(SvgFrame& frame) {
        frame.add_tour(tour_);
    }

protected:
    Polygon domain_;
    vector<Point> tour_;
};

class GridPlanner: public Planner
{
public:
    GridPlanner(const Polygon& domain): Planner(domain) {}

    std::vector<Point> calc_tour() override {
        build_grid();
        filter_grid();
        return grid_;
    }

    void vis(SvgFrame& frame) override {
        Planner::vis(frame);

        frame.add_grid(grid_);
    }

protected:

    void filter_grid() {
        boost::geometry::model::linestring<Point> boost_poly(domain_.vertices_begin(), domain_.vertices_end());
        boost_poly.push_back(*boost_poly.begin());

        std::vector<Point> vertices;
        for (auto& p : grid_) {
            if (boost::geometry::distance(p, boost_poly) <= VIS_RADIUS * sqrt(2.0) ||
                domain_.bounded_side(p) == cgal::ON_BOUNDED_SIDE)
            {
                vertices.push_back(p);
            }
        }

        grid_ = vertices;
    }

    void build_grid() {
        BoundBox box = domain_.bbox();

        int radius = ceil(std::max(box.xmax() - box.xmin(), box.ymax() - box.ymin()));
        int d = 2 * VIS_RADIUS;
        int count = (radius + d - 1) / d;

        std::cerr << radius << " " << count << std::endl;

        std::vector<Point> raw_grid;
        cgal::points_on_square_grid_2(count * d, SQR(2 * count + 1), std::back_inserter(grid_),Creator());
    }

    void build_graph() {
        Graph graph(grid_);

        for (int i = 0; i < grid_.size(); i++) {
            for (int j = 0; j < i; j++) {
                if (cgal::squared_distance(grid_[i], grid_[j]) == SQR(2 * VIS_RADIUS)) {
                    graph.add_edge(i, j, 2 * VIS_RADIUS);
                }
            }
        }

        for (const auto& edges : graph_.adj()) {
            assert(edges.size() <= 4);
            assert(edges.size() > 0);
        }

        graph_ = graph;
    }

    std::vector<Point> grid_;
    Graph graph_;
};

class GreedyGridPlanner: public GridPlanner
{
};


class MstGridPlanner: public GridPlanner
{
public:
    MstGridPlanner(const Polygon& domain): GridPlanner(domain) {}

    std::vector<Point> calc_tour() override {
        build_grid();
        filter_grid();
        build_graph();

        std::cerr << "Calculating tour" << std::endl;

        std::vector<int> used(graph_.vertices().size());
        std::priority_queue<std::pair<int, int>,
            std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>> > d;

        d.emplace(0, 0);

        std::vector<Point> tour;
        while (!d.empty()) {
            auto dv = d.top();
            d.pop();
            if (used[dv.second]) {
                continue;
            }

            tour.push_back(graph_.vertices()[dv.second]);
            used[dv.second] = true;

            for (auto e : graph_.adj(dv.second)) {
                if (!used[e.v]) {
                    d.emplace(e.weight, e.v);
                }
            }
        }

        std::cerr << "Tour finished" << std::endl;
        return tour;
    }

private:

};