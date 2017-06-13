#include <iostream>
#include <cassert>
#include <fstream>
#include <vector>

#include "vis.hpp"
#include "geom.hpp"
#include "planner.hpp"

using namespace CGAL;
using namespace std;

namespace bg = boost::geometry;

int main(int argc, char** argv) {

    Polygon domain = random_poly(15, 900, 777);

    MstGridPlanner planner(domain);

    SvgFrame output(2000, "output.svg");
    output.add_domain(domain);

    auto tour = planner.calc_tour();
    planner.vis(output);

    return 0;
}