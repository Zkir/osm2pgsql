#ifndef OSM2PGSQL_GEN_RIVERS_HPP
#define OSM2PGSQL_GEN_RIVERS_HPP

/**
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This file is part of osm2pgsql (https://osm2pgsql.org/).
 *
 * Copyright (C) 2006-2025 by the osm2pgsql developer community.
 * For a full list of authors see the git log.
 */

#include "gen-base.hpp"
#include "geom.hpp"
#include "osmtypes.hpp"

#include <cstddef>
#include <string_view>
#include <vector>

class params_t;
class pg_conn_t;

/// The data for a graph edge in the waterway network.
struct edge_t
{
    // All the points in this edge
    geom::linestring_t points;

    // Edges can be made from (part) of one or more OSM ways, this is the id
    // of one of them.
    osmid_t id = 0;

    // The width of the river along this edge
    double width = 0.0;

    // The rank of the river segment
    double rank = 0.0;
};

inline bool operator<(edge_t const &a, edge_t const &b) noexcept
{
    assert(a.points.size() > 1 && b.points.size() > 1);
    if (a.points[0] == b.points[0]) {
        return a.points[1] < b.points[1];
    }
    return a.points[0] < b.points[0];
}

inline bool operator<(edge_t const &a, geom::point_t b) noexcept
{
    assert(!a.points.empty());
    return a.points[0] < b;
}

inline bool operator<(geom::point_t a, edge_t const &b) noexcept
{
    assert(!b.points.empty());
    return a < b.points[0];
}

class gen_rivers_t : public gen_base_t
{
public:
    gen_rivers_t(pg_conn_t *connection, bool append, params_t *params);

    void process() override;

    std::string_view strategy() const noexcept override { return "rivers"; }

private:
    void get_stats();
    void break_cycles(std::vector<edge_t> &edges);

    std::size_t m_timer_area;
    std::size_t m_timer_prep;
    std::size_t m_timer_get;
    std::size_t m_timer_sort;
    std::size_t m_timer_net;
    std::size_t m_timer_break_cycles;
    std::size_t m_timer_remove;
    std::size_t m_timer_width;
    std::size_t m_timer_rank;
    std::size_t m_timer_write;

    std::size_t m_num_waterways = 0;
    std::size_t m_num_points = 0;
};

#endif // OSM2PGSQL_GEN_RIVERS_HPP


