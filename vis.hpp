#pragma once

#include <fstream>
#include <string>

#include "geom.hpp"

class SvgFrame
{
public:
    SvgFrame(int size, std::string filename): svg_out_(filename), mapper_(svg_out_, size, size) {}

    SvgFrame& add_domain(const Polygon& domain) {
        boost::geometry::model::ring<Point> boost_poly(domain.vertices_begin(), domain.vertices_end());

        mapper_.add(boost_poly);
        mapper_.map(boost_poly, "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:2");

        return *this;
    }

    SvgFrame& add_grid(const std::vector<Point> grid) {
        mapper_.add(grid);
        mapper_.map(grid, "opacity:1.0;fill:rgb(255,0,0);stroke:rgb(255,0,0);stroke-width:5", 1);

        return *this;
    }
private:
    std::ofstream svg_out_;
    boost::geometry::svg_mapper<Point> mapper_;
};