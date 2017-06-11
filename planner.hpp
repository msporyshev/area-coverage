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

    virtual const std::vector<Point>& calc_tour() = 0;
    virtual void vis(SvgFrame& frame) {
        assert(tour_.size() > 1);

        frame.add_start_point(tour_[0]);
        frame.add_tour(tour_);
    }

protected:
    Polygon domain_;
    std::vector<Point> tour_;
};

class GridPlanner: public Planner
{
public:
    GridPlanner(const Polygon& domain): Planner(domain) {}

    const std::vector<Point>& calc_tour() override = 0;

    void vis(SvgFrame& frame) override {
        Planner::vis(frame);

        frame.add_grid(grid_);
    }

protected:

    // void filter_grid();

    // void build_grid();

    void build_graph();

    std::vector<Point> grid_;
    Graph graph_;
};

class GreedyGridPlanner: public GridPlanner
{
public:
    GreedyGridPlanner(const Polygon& domain): GridPlanner(domain) {}

    const std::vector<Point>& calc_tour() override;
};


class MstGridPlanner: public GridPlanner
{
public:
    MstGridPlanner(const Polygon& domain): GridPlanner(domain) {}

    const std::vector<Point>& calc_tour() override;

    void vis(SvgFrame& frame) override {
        // Planner::vis(frame);

        frame.add_grid(grid_);
        frame.add_grid(x2grid_);
        for (auto& e : edges_) {
            frame.add_edge(x2graph_, e.from, e.to);
        }
    }

protected:
    std::vector<Point> x2grid_;
    Graph x2graph_;
    std::vector<Edge> edges_;
};