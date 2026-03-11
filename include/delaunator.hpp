#pragma once

#include <limits>
#include <vector>
#include <unordered_set>

namespace delaunator {
constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();

class Delaunator {

public:
    std::vector<double> const coords;
    std::vector<std::size_t> triangles;
    std::vector<std::size_t> halfedges;
    std::vector<std::size_t> hull_prev;
    std::vector<std::size_t> hull_next;
    std::vector<std::size_t> hull_tri;
    std::size_t hull_start;

    explicit Delaunator(std::vector<double> in_coords);

    double get_hull_area() const;

    template<typename Callback>
    void for_each_hull_triangle(Callback&& cb) const {
        size_t e = hull_start;
        do {
            cb(hull_tri[e], e);  // triangle index, hull edge point
            e = hull_next[e];
        } while (e != hull_start);
    }

    std::unordered_set<size_t> hull_neighbor_triangles;
    std::vector<size_t> hull_search_candidates;

    void compute_hull_neighbors(size_t degrees=5);
    void compute_hull_search_candidates();

private:
    std::vector<std::size_t> m_hash;
    double m_center_x;
    double m_center_y;
    std::size_t m_hash_size;
    std::vector<std::size_t> m_edge_stack;

    std::size_t legalize(std::size_t a);
    std::size_t hash_key(double x, double y) const;
    std::size_t add_triangle(
        std::size_t i0,
        std::size_t i1,
        std::size_t i2,
        std::size_t a,
        std::size_t b,
        std::size_t c);
    void link(std::size_t a, std::size_t b);
};

} //namespace delaunator
