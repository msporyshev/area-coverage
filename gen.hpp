#pragma once

#include "geom.hpp"

std::vector<Point> gen_sq_grid(const Point& top_left, double dist, int axis_count) {
    std::vector<Point> res;
    for (int i = 0; i < axis_count; i++) {
        for (int j = 0; j < axis_count; j++) {
            res.emplace_back(top_left.x() + j * dist, top_left.y() + i * dist);
        }
    }

    return res;
}