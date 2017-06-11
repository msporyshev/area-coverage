#include "graph.hpp"

#include <algorithm>
#include <iostream>
#include <queue>

using std::vector;
using std::queue;

vector<Point> StartVertexSearcher::get_tour_to(int v) {
    vector<int> tour;
    int cur = v;
    while (prev_[cur] != sv_) {
        tour.push_back(cur);
        cur = prev_[cur];
    }

    std::reverse(tour.begin(), tour.end());

    return graph_.get_2d_tour(tour);
}

const std::vector<double>& BfsSearcher::calc_distances() {
    const auto& adj_list = graph_.adj();
    queue<int> q;

    q.push(sv_);
    distances_.assign(adj_list.size(), -1.0);
    prev_.assign(adj_list.size(), -1);
    distances_[sv_] = 0;


    while (!q.empty()) {
        int from = q.front();
        q.pop();

        for (auto to : adj_list[from]) {
            if (distances_[to.v] != -1.0) {
                prev_[to.v] = from;
                distances_[to.v] = distances_[from] + 1;
                q.push(to.v);
            }
        }
    }

    return distances_;
}
