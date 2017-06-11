#include "graph.hpp"

#include <algorithm>
#include <iostream>
#include <queue>

using std::vector;
using std::queue;

vector<int> StartVertexSearcher::get_tour_to(int v) {
    vector<int> tour;
    tour.push_back(v);
    int cur = v;

    while (prev_[cur] != sv_) {
        assert(prev_[cur] != -1);
        tour.push_back(prev_[cur]);
        cur = prev_[cur];
    }

    std::reverse(tour.begin(), tour.end());

    return tour;
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

        for (auto e : adj_list[from]) {

            if (distances_[e.to] < 0) {
                prev_[e.to] = from;
                distances_[e.to] = distances_[from] + 1;
                q.push(e.to);
            }
        }
    }

    return distances_;
}
