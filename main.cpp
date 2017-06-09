#include <iostream>
#include <cassert>
#include <fstream>
#include <vector>

#include "vis.hpp"
#include "geom.hpp"

using namespace CGAL;
using namespace std;

namespace bg = boost::geometry;

int main(int argc, char** argv)
{
    std::vector<Point> grid;
    points_on_square_grid_2( 1000.0, 900, std::back_inserter(grid),Creator());

    Polygon domain = random_poly(15, 900, 777);

    SvgFrame output(1000, "output.svg");
    output.add_grid(grid).add_domain(domain);

    return 0;
}