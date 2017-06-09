#include "geom.hpp"

#include <vector>

using namespace std;
namespace cgal = CGAL;

Polygon random_poly(int point_count, int field_size, int seed) {
    vector<Point> point_set;

    cgal::Random num_gen(seed);
    PointGenerator gen(field_size, num_gen);

    cgal::copy_n_unique(gen, point_count, back_inserter(point_set));

    Polygon res;
    cgal::random_polygon_2(point_set.size(), back_inserter(res), point_set.begin());

    return res;
}
