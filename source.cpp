#include <iostream>
#include <cassert>
#include <fstream>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/multi_point.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/algorithm.h>

using namespace CGAL;
using namespace std;

typedef Simple_cartesian<int> K;

struct Point: K::Point_2
{
    Point(): K::Point_2() {}
    Point(int x, int y): K::Point_2(x, y) {}
    void set_x(int x) {
        *this = Point(x, this->y());
    }
    void set_y(int y) {
        *this = Point(this->x(), y);
    }
};

typedef CGAL::Polygon_2<K, std::vector<Point> > Polygon;

typedef Creator_uniform_2<int,Point> Creator;
typedef CGAL::Random_points_in_square_2<Point, Creator> PointGenerator;

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Point, int, cs::cartesian, x, y, set_x, set_y);
BOOST_GEOMETRY_REGISTER_MULTI_POINT(std::vector<Point>);

typedef boost::geometry::model::polygon<Point> polygon_t;

namespace bg = boost::geometry;

int main(int argc, char** argv)
{
    assert(argc > 1);

    ifstream domain_input(argv[1]);
    string polygon_str;

    getline(domain_input, polygon_str);

    polygon_t domain;
    bg::read_wkt(polygon_str, domain);


    std::vector<Point> grid;
    points_on_square_grid_2( 1000.0, 1000, std::back_inserter(grid),Creator());

    std::ofstream svg("my_map.svg");
    boost::geometry::svg_mapper<Point> mapper(svg, 1000, 1000);

    mapper.add(domain);
    mapper.add(grid);

    mapper.map(domain, "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:2");
    mapper.map(grid, "opacity:0.4;fill:none;stroke:rgb(212,0,0);stroke-width:5");

    return 0;
}