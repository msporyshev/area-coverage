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
    std::vector<Point> tour_;
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

    void filter_grid();

    void build_grid();

    void build_graph();

    std::vector<Point> grid_;
    Graph graph_;
};

class GreedyGridPlanner: public GridPlanner
{
public:
    std::vector<Point> calc_tour() override;
};


class MstGridPlanner: public GridPlanner
{
public:
    MstGridPlanner(const Polygon& domain): GridPlanner(domain) {}

    std::vector<Point> calc_tour() override;

private:

};