#pragma once
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
#include <CGAL/random_polygon_2.h>
#include <CGAL/algorithm.h>


typedef CGAL::Simple_cartesian<int> K;

struct Point: K::Point_2
{
    Point(): K::Point_2() {}
    Point(int x, int y): K::Point_2(x, y) {}
    Point(const K::Point_2& cgal_pt): K::Point_2(cgal_pt) {}

    void set_x(int x) {
        *this = Point(x, this->y());
    }
    void set_y(int y) {
        *this = Point(this->x(), y);
    }
};

typedef CGAL::Polygon_2<K, std::vector<typename K::Point_2> > Polygon;

typedef K::Vector_2 Vector;
typedef CGAL::Creator_uniform_2<int, Point> Creator;
typedef CGAL::Random_points_in_square_2<Point, Creator> PointGenerator;
typedef CGAL::Bbox_2 BoundBox;
typedef K::Circle_2 Circle;

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Point, int, cs::cartesian, x, y, set_x, set_y);
BOOST_GEOMETRY_REGISTER_MULTI_POINT(std::vector<Point>);

typedef boost::geometry::model::polygon<Point> BoostPolygon;

Polygon random_poly(int point_count, int field_size, int seed = 0);
