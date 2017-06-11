#include "planner.hpp"
#include "graph.hpp"
#include "gen.hpp"

#include <iostream>
#include <vector>
#include <tuple>
#include <queue>

using std::vector;

namespace {

vector<Point> filter_grid(const Polygon& domain_, double dist, const vector<Point>& grid_) {
    boost::geometry::model::linestring<Point> boost_poly(domain_.vertices_begin(), domain_.vertices_end());
    boost_poly.push_back(*boost_poly.begin());

    std::vector<Point> vertices;
    for (auto& p : grid_) {
        if (boost::geometry::distance(p, boost_poly) <= dist ||
            domain_.bounded_side(p) == cgal::ON_BOUNDED_SIDE)
        {
            vertices.push_back(p);
        }
    }

    return vertices;
}

vector<Point> build_bounding_grid(const Polygon& domain, double radius) {
    BoundBox box = domain.bbox();

    int d = 2 * radius;
    int count = ceil(std::max(box.xmax() - box.xmin(), box.ymax() - box.ymin()) / d);
    auto grid = gen_sq_grid(Point(box.xmin(), box.ymin()), d, count);

    grid = filter_grid(domain, radius * sqrt(2.0), grid);

    return grid;
}


Graph build_4c_grid_graph(const vector<Point>& vertices, int dist) {
    Graph graph(vertices);

    for (int i = 0; i < vertices.size(); i++) {
        for (int j = 0; j < i; j++) {
            if (cgal::squared_distance(vertices[i], vertices[j]) == SQR(dist)) {
                graph.add_edge(i, j, dist);
            }
        }
    }

    for (const auto& edges : graph.adj()) {
        assert(edges.size() <= 4);
        assert(edges.size() > 0);
    }

    return graph;
}


} // namespace

// void GridPlanner::filter_grid() {
//     boost::geometry::model::linestring<Point> boost_poly(domain_.vertices_begin(), domain_.vertices_end());
//     boost_poly.push_back(*boost_poly.begin());

//     std::vector<Point> vertices;
//     for (auto& p : grid_) {
//         if (boost::geometry::distance(p, boost_poly) <= VIS_RADIUS * sqrt(2.0) ||
//             domain_.bounded_side(p) == cgal::ON_BOUNDED_SIDE)
//         {
//             vertices.push_back(p);
//         }
//     }

    // grid_ = vertices;
// }


void GridPlanner::build_graph() {
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

const std::vector<Point>& GreedyGridPlanner::calc_tour() {
    grid_ = build_bounding_grid(domain_, VIS_RADIUS);
    build_graph();

    std::vector<int> tour;
    tour.push_back(0);
    int vcount = graph_.vertices().size();

    std::vector<int> used(vcount);
    used[0] = true;

    for (int i = 0; i < vcount - 1; i++) {
        BfsSearcher searcher(graph_, tour.back());
        std::vector<double> distances = searcher.calc_distances();

        double mind = 1e9;
        int v = -1;
        for (int i = 0; i < vcount; i++) {
            if (!used[i] && mind > distances[i]) {
                mind = distances[i];
                v = i;
            }
        }

        used[v] = true;

        auto tour_to_v = searcher.get_tour_to(v);

        std::copy(tour_to_v.begin(), tour_to_v.end(), std::back_inserter(tour));
    }

    tour_ = graph_.get_2d_tour(tour);
    return tour_;
}

const std::vector<Point>& MstGridPlanner::calc_tour() {
    grid_ = build_bounding_grid(domain_, VIS_RADIUS);

    BoundBox box = domain_.bbox();

    int d = 2 * VIS_RADIUS;
    int count = ceil(std::max(box.xmax() - box.xmin(), box.ymax() - box.ymin()) / d);
    // grid_ = gen_sq_grid(Point(box.xmin(), box.ymin()), d, count);

    x2grid_ = gen_sq_grid(Point(box.xmin() + VIS_RADIUS, box.ymin() + VIS_RADIUS), 2 * d, count / 2);
    x2grid_ = filter_grid(domain_, d, x2grid_);


    // filter_grid();
    graph_ = build_4c_grid_graph(grid_, d);

    x2graph_ = build_4c_grid_graph(x2grid_, 2 * d);

    std::vector<int> used(x2graph_.vertices().size());
    std::priority_queue<Edge,
        std::vector<Edge>, std::greater<Edge> > dists;

    dists.emplace(-1, 0, 0);

    std::vector<Point> tour;
    while (!dists.empty()) {
        auto cur_edge = dists.top();
        dists.pop();
        if (used[cur_edge.to]) {
            continue;
        }

        // tour.push_back(x2graph_.vertices()[cur_edge.second]);
        if (cur_edge.to != 0) {
            edges_.push_back(cur_edge);
        }

        used[cur_edge.to] = true;

        for (auto e : x2graph_.adj(cur_edge.to)) {
            if (!used[e.to]) {
                dists.push(e);
            }
        }
    }

    // tour_ = tour;
    return tour_;
}
