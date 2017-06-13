#pragma once

#include <fstream>
#include <string>

#include "geom.hpp"
#include "graph.hpp"

class SvgFrame
{
public:
    SvgFrame(int size, std::string filename): size_(size), svg_out_(filename), mapper_(svg_out_, size, size) {}
    SvgFrame(SvgFrame&& frame)
            : size_(frame.size_)
            , svg_out_(std::move(frame.svg_out_))
            , mapper_(svg_out_, size_, size_)
    {}

    SvgFrame& add_tour(const std::vector<Point>& tour) {
        boost::geometry::model::linestring<Point> boost_poly(tour.begin(), tour.end());
        mapper_.add(boost_poly);
        mapper_.map(boost_poly, "opacity:1.0;fill:rgb(255,0,0);stroke:rgb(255,0,0);stroke-width:2");

        return *this;
    }

    SvgFrame& add_arc(const Graph& graph, int i, int j) {
        return *this;
    }

    SvgFrame& add_edge(const Graph& graph, int i, int j) {
        boost::geometry::model::segment<Point> segment(graph.vertices()[i], graph.vertices()[j]);

        mapper_.add(segment);
        mapper_.map(segment, "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:2");
        return *this;
    }

    SvgFrame& add_graph(const Graph& graph) {
        add_grid(graph.vertices());
        for (auto& adj_list : graph.adj()) {
            for (auto& e : adj_list) {
                add_edge(graph, e.from, e.to);
            }
        }
        return *this;
    }

    SvgFrame& add_start_point(const Point& sp) {
        mapper_.add(sp);
        mapper_.map(sp, "opacity:1.0;fill:rgb(0,255,0);stroke:rgb(255,0,0);stroke-width:5", 10);

        return *this;
    }

    SvgFrame& add_domain(const Polygon& domain) {
        boost::geometry::model::ring<Point> boost_poly(domain.vertices_begin(), domain.vertices_end());

        mapper_.add(boost_poly);
        mapper_.map(boost_poly, "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:2");

        return *this;
    }

    SvgFrame& add_grid(const SqGrid& grid) {
        return add_grid(grid.vertices());
    }

    SvgFrame& add_grid(const std::vector<Point> grid) {
        mapper_.add(grid);
        mapper_.map(grid, "opacity:1.0;fill:rgb(255,0,0);stroke:rgb(255,0,0);stroke-width:5", 1);

        return *this;
    }

    SvgFrame& add(const Graph& graph) {
        return add_graph(graph);
    }

    SvgFrame& add(const Polygon& domain) {
        return add_domain(domain);
    }

    SvgFrame& add(const std::vector<Point>& grid) {
        return add_grid(grid);
    }

    SvgFrame& add(const SqGrid& grid) {
        return add_grid(grid);
    }

    SvgFrame& add(const Point& p) {
        return add_start_point(p);
    }

    template<typename T>
    SvgFrame& operator<<(const T& obj) {
        return add(obj);
    }

private:
    int size_;
    std::ofstream svg_out_;
    boost::geometry::svg_mapper<Point> mapper_;
};