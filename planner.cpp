#include "common.hpp"
#include "planner.hpp"
#include "graph.hpp"
#include "gen.hpp"
#include "log.hpp"

#include <iostream>
#include <vector>
#include <tuple>
#include <set>
#include <queue>
#include <map>

using std::set;
using std::vector;
using std::map;
using std::pair;

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

static void add_shift_edge(const Point& p, const SqGrid& grid, Pair<Vector> delta,  Graph& out) {
    Point a = p + delta[0];
    Point b = p + delta[1];

    out.add_edge(grid.get_index(a), grid.get_index(b), boost::geometry::distance(a, b));
}

static std::vector<int> get_tour(const Graph& one_cycle_graph) {
    int sv = -1;
    for (int i = 0; i < one_cycle_graph.vertices().size(); i++) {
        if (!one_cycle_graph.adj(i).empty()) {
            sv = i;
            break;
        }
    }

    assert(sv >= 0);

    int v = sv;
    vector<int> used(one_cycle_graph.vertices().size());
    vector<int> tour;
    do {
        tour.push_back(v);

        for (auto e: one_cycle_graph.adj(v)) {
            if (!used[e.to]) {
                v = e.to;
                break;
            }
        }
        used[v] = true;
    } while (v != sv);

    tour.push_back(sv);

    return tour;
}

const std::vector<Point>& MstGridPlanner::calc_tour() {
    grid_ = build_bounding_grid(domain_, VIS_RADIUS);

    BoundBox box = domain_.bbox();

    int d = 2 * VIS_RADIUS;
    int count = ceil(std::max(box.xmax() - box.xmin(), box.ymax() - box.ymin()) / d);
    grid_ = gen_sq_grid(Point(box.xmin(), box.ymin()), d, count);

    x2grid_ = gen_sq_grid(Point(box.xmin() + VIS_RADIUS, box.ymin() + VIS_RADIUS), 2 * d, count / 2);
    x2grid_ = filter_grid(domain_, d / 2, x2grid_);


    graph_ = build_4c_grid_graph(grid_, d);

    x2graph_ = build_4c_grid_graph(x2grid_, 2 * d);

    DebugFrame("x2graph") << x2graph_ << domain_;

    std::vector<int> used(x2graph_.vertices().size());
    std::priority_queue<Edge,
        std::vector<Edge>, std::greater<Edge> > dists;

    dists.emplace(-1, 0, 0);

    mst_.set_vertices(x2grid_);

    SqGrid sq_grid(Point(box.xmin(), box.ymin()), d, count);
    Graph euler_graph(grid_);

    std::vector<Point> tour;
    int vr = VIS_RADIUS;

    Pair<Vector> left = {Vector{-1, -1} * vr, Vector{-1, 1} * vr};
    Pair<Vector> right = {Vector{1, -1} * vr, Vector{1, 1} * vr};
    Pair<Vector> up = {Vector{-1, 1} * vr, Vector{1, 1} * vr};
    Pair<Vector> down = {Vector{-1, -1} * vr, Vector{1, -1} * vr};


    while (!dists.empty()) {
        auto cur_edge = dists.top();
        dists.pop();
        if (used[cur_edge.to]) {
            continue;
        }

        if (cur_edge.to != 0) {
            mst_.add_edge(cur_edge);

            Point a = mst_.vertices()[cur_edge.from];
            Point b = mst_.vertices()[cur_edge.to];

            Point p((a.x() + b.x()) / 2, (a.y() + b.y()) / 2);
            if (a.x() == b.x()) {
                add_shift_edge(p, sq_grid, left, euler_graph);
                add_shift_edge(p, sq_grid, right, euler_graph);
            } else {
                add_shift_edge(p, sq_grid, up, euler_graph);
                add_shift_edge(p, sq_grid, down, euler_graph);
            }
        }

        used[cur_edge.to] = true;

        for (auto e : x2graph_.adj(cur_edge.to)) {
            if (!used[e.to]) {
                dists.push(e);
            }
        }
    }

    map<Pair<int>, Pair<Vector> > shift_by_dir = {
        {{-1, 0}, left},
        {{1, 0}, right},
        {{0, 1}, up},
        {{0, -1}, down}
    };

    for (int i = 0; i < mst_.vertices().size(); i++) {
        set<Pair<int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (auto& edge : mst_.adj(i)) {
            Point a = mst_.vertices()[edge.from];
            Point b = mst_.vertices()[edge.to];
            Pair<int> dir = {(b.x() - a.x()) / (2 * d), (b.y() - a.y()) / (2 * d)};

            dirs.erase(dir);
        }

        for (auto& dir : dirs) {
            add_shift_edge(mst_.vertices()[i], sq_grid, shift_by_dir[dir], euler_graph);
        }
    }

    auto vertex_tour = get_tour(euler_graph);
    tour_ = euler_graph.get_2d_tour(vertex_tour);

    QuadTree qtree(domain_, d, 2);
    DebugFrame("euler_graph") << domain_ << grid_ << euler_graph << mst_;
    DebugFrame("euler_tour") << domain_ << euler_graph;
    (DebugFrame("qtree_tour") << domain_ << qtree << qtree.grid()).add_tour(tour_);

    return tour_;
}
