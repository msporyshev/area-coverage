#pragma once

#include "graph.hpp"
#include "common.hpp"

#include <cmath>

struct QTreeNode {
    QTreeNode() {}
    QTreeNode(const Point& tl, int dist, int count): tl_(tl), dist_(dist), count_(count) {}

    std::vector<QTreeNode> children_;
    Point tl_;
    int dist_;
    int count_;
};

class QuadTree
{
public:
    static QTreeNode build_tree(const Point& tl, int dist, int count, int depth) {
        QTreeNode res(tl, dist, count);
        if (count <= 2 || depth == 1) {
            return res;
        }

        int m = count / 2;
        Vector dirs[] = {Vector{0, 0}, Vector{0, 1}, Vector{1, 0}, Vector{1, 1}};
        for (int i = 0; i < 4; i++) {
            res.children_.push_back(build_tree(tl + dirs[i] * m * dist, dist, count / 2, depth - 1));
        }

        return res;
    }

    static SqGrid build_grid(const Polygon& domain, int dist) {
        BoundBox box = domain.bbox();

        int count = ceil(std::max(box.xmax() - box.xmin(), box.ymax() - box.ymin()) / (double)dist);
        int k = ceil(log2(1.0 * count));
        count = (1 << k) + 1;
        return SqGrid(Point(box.xmin(), box.ymin()), dist, count);
    }

    QuadTree(const Polygon& domain, int dist, int depth = -1)
            : grid_(build_grid(domain, dist))
            , domain_(domain)
            , root_(build_tree(grid_[0][0], dist, grid_.axis_count() - 1, depth))
    {
    }

    const QTreeNode& root() const {
        return root_;
    }

    const SqGrid& grid() const {
        return grid_;
    }

private:
    SqGrid grid_;
    Polygon domain_;
    QTreeNode root_;
};