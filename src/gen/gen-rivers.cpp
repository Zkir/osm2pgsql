/**
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This file is part of osm2pgsql (https://osm2pgsql.org/).
 *
 * Copyright (C) 2006-2025 by the osm2pgsql developer community.
 * For a full list of authors see the git log.
 */

#include "gen-rivers.hpp"

#include "geom-functions.hpp"
#include "hex.hpp"
#include "logging.hpp"
#include "params.hpp"
#include "pgsql.hpp"
#include "projection.hpp"
#include "util.hpp"
#include "wkb.hpp"

#include <cmath>
#include <cassert>
#include <limits>
#include <map>
#include <unordered_map>
#include <vector>

gen_rivers_t::gen_rivers_t(pg_conn_t *connection, bool append, params_t *params)
: gen_base_t(connection, append, params), m_timer_area(add_timer("area")),
  m_timer_prep(add_timer("prep")), m_timer_get(add_timer("get")),
  m_timer_sort(add_timer("sort")), m_timer_net(add_timer("net")),
  m_timer_break_cycles(add_timer("break_cycles")),
  m_timer_remove(add_timer("remove")), m_timer_width(add_timer("width")),
  m_timer_rank(add_timer("rank")), m_timer_write(add_timer("write"))
{
    check_src_dest_table_params_exist();

    params->check_identifier_with_default("src_areas", "waterway_areas");
    params->check_identifier_with_default("id_column", "way_id");
    params->check_identifier_with_default("width_column", "width");
    params->check_identifier_with_default("name_column", "name");

    params->set("qualified_src_areas",
                qualified_name(get_params().get_string("schema"),
                               get_params().get_string("src_areas")));
}

namespace {

void follow_chain_and_set_width(
    edge_t const &edge, std::vector<edge_t> *edges,
    std::map<geom::point_t, uint8_t> const &node_order,
    geom::linestring_t *seen)
{
    assert(!edge.points.empty());

    auto const seen_it =
        std::find(seen->cbegin(), seen->cend(), edge.points[0]);
    if (seen_it != seen->cend()) {
        return; // loop detected
    }

    seen->push_back(edge.points[0]);

    assert(edge.points.size() > 1);
    auto const next_point = edge.points.back();
    if (node_order.at(next_point) > 1) { // It's not an endpoint
        auto const [s, e] =
            std::equal_range(edges->begin(), edges->end(), next_point);

        if (std::next(s) == e) { // Only one downstream edge
            // Replacement only if child's width is 0/NULL and parent has width
            if ((s->width == 0.0 || std::isnan(s->width)) && edge.width > 0.0) {
                s->width = edge.width;
                // Recurse only if a change was made
                follow_chain_and_set_width(*s, edges, node_order, seen);
            }
        } else { // Multiple downstream edges (a split)
            // Find a downstream edge that is a continuation of the same way.
            edge_t *continuation = nullptr;
            for (auto it = s; it != e; ++it) {
                if (it->id == edge.id) {
                    continuation = &*it;
                    break;
                }
            }

            if (continuation) {
                // Found a continuation of the same way. Propagate width and
                // recurse along this path.
                if ((continuation->width == 0.0 || std::isnan(continuation->width)) && edge.width > 0.0) {
                    continuation->width = edge.width;
                    follow_chain_and_set_width(*continuation, edges, node_order, seen);
                }
            } else {
                // No continuation of the same way found. This is a "real"
                // fork. Fall back to old logic: check if ALL children have
                // no width.
                bool all_children_have_no_width = true;
                for (auto it_check = s; it_check != e; ++it_check) {
                    if (it_check->width > 0.0) {
                        all_children_have_no_width = false;
                        break;
                    }
                }

                // If condition is met, propagate to the first child and
                // recurse ONLY on that path.
                if (all_children_have_no_width && edge.width > 0.0) {
                    if (s != e) { // Check that there is at least one child
                        s->width = edge.width;
                        // Recurse on the updated child. The original `seen`
                        // is passed.
                        follow_chain_and_set_width(*s, edges, node_order, seen);
                    }
                }
            }
        }
    }
}

void assemble_edge(edge_t *edge, std::vector<edge_t> *edges,
                   std::map<geom::point_t, uint8_t> const &node_order)
{
    assert(edge);
    assert(edges);
    while (true) {
        assert(edge->points.size() > 1);
        geom::point_t const next_point = edge->points.back();

        auto const count = node_order.at(next_point);
        if (count != 2) {
            return;
        }

        auto const [s, e] =
            std::equal_range(edges->begin(), edges->end(), next_point);

        if (s == e) {
            return;
        }
        assert(e == std::next(s));

        auto const it = s;
        if (it->points.size() == 1 || &*it == edge) {
            return;
        }
        
        if (it->points[0] != next_point) {
            return;
        }
        assert(it != edges->end());

        edge->width = std::max(edge->width, it->width);

        if (it->points.size() == 2) {
            edge->points.push_back(it->points.back());
            it->points.resize(1);
            it->points.shrink_to_fit();
        } else {
            edge->points.insert(edge->points.end(),
                                std::next(it->points.begin()),
                                it->points.end());
            it->points.resize(1);
            it->points.shrink_to_fit();
            return;
        }
    }
}

std::string const &get_name(
    std::unordered_map<osmid_t, std::string> const &names, osmid_t id)
{
    static std::string const empty;
    auto const it = names.find(id);
    if (it == names.end()) {
        return empty;
    }
    return it->second;
}

} // anonymous namespace

void gen_rivers_t::break_cycles(std::vector<edge_t> &edges)
{
    enum class node_state { white, gray, black };

    std::map<geom::point_t, std::vector<edge_t *>> adj;
    std::map<geom::point_t, node_state> states;

    for (auto &edge : edges) {
        if (edge.points.size() > 1) {
            adj[edge.points.front()].push_back(&edge);
            states[edge.points.front()] = node_state::white;
            states[edge.points.back()] = node_state::white;
        }
    }

    std::vector<geom::point_t> path;

    std::function<void(geom::point_t const &)> visitor =
        [&](geom::point_t const &u) {
        states[u] = node_state::gray;
        path.push_back(u);

        if (adj.count(u)) {
            for (auto *edge : adj.at(u)) {
                if (edge->points.size() < 2) {
                    continue;
                }
                geom::point_t const v = edge->points.back();

                if (states.find(v) == states.end()) {
                    continue;
                }

                if (states.at(v) == node_state::gray) {
                    log_debug("Cycle detected. Analyzing...");

                    auto cycle_start_it = std::find(path.begin(), path.end(), v);
                    if (cycle_start_it == path.end()) {
                        continue;
                    }

                    edge_t *closing_edge = edge;
                    edge_t *weakest_edge_in_path = nullptr;
                    double min_width = std::numeric_limits<double>::max();

                    log_debug("--- Cycle Path ---");
                    for (auto it = cycle_start_it; it != path.end(); ++it) {
                        geom::point_t const path_node_u = *it;
                        auto next_it = std::next(it);

                        if (next_it != path.end()) {
                            geom::point_t const path_node_v = *next_it;
                            if(adj.count(path_node_u)) {
                                for(auto *e : adj.at(path_node_u)) {
                                    if(e->points.back() == path_node_v) {
                                        log_debug("  - Path edge osm_id={}, width={}", e->id, e->width);
                                        if (weakest_edge_in_path == nullptr || e->width < min_width) {
                                            min_width = e->width;
                                            weakest_edge_in_path = e;
                                        }
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    log_debug("  - Closing edge osm_id={}, width={}", closing_edge->id, closing_edge->width);
                    log_debug("--------------------");

                    edge_t *weakest_edge = weakest_edge_in_path;
                    // If all widths are 0, or closing edge is the actual weakest
                    if (weakest_edge == nullptr || (closing_edge->width <= min_width && closing_edge->points.size() > 1)) {
                        weakest_edge = closing_edge;
                    }
                    
                    log_debug("  - Selected weakest edge for breaking: osm_id={}, width={}, points.size()={}",
                              weakest_edge->id, weakest_edge->width, weakest_edge->points.size());

                    if (weakest_edge && weakest_edge->points.size() > 1) {
                        log_gen("Breaking cycle by removing edge with OSM ID {} and width {}.",
                                weakest_edge->id, weakest_edge->width);
                        weakest_edge->points.resize(1);
                    }

                } else if (states.at(v) == node_state::white) {
                    visitor(v);
                }
            }
        }

        path.pop_back();
        states[u] = node_state::black;
    };

    for (auto const& [node, state] : states) {
        if (state == node_state::white) {
            path.clear();
            visitor(node);
        }
    }
}

/// Get some stats from source table
void gen_rivers_t::get_stats()
{
    auto const result =
        dbexec("SELECT count(*), sum(ST_NumPoints(geom)) FROM {src}");

    m_num_waterways = strtoul(result.get_value(0, 0), nullptr, 10);
    m_num_points = strtoul(result.get_value(0, 1), nullptr, 10);

    log_gen("Found {} waterways with {} points.", m_num_waterways,
            m_num_points);
}

void gen_rivers_t::process()
{
    log_gen("Calculate waterway area width...");
    timer(m_timer_area).start();
    dbexec(R"(UPDATE {qualified_src_areas} SET width =)"
           R"( (ST_MaximumInscribedCircle("{geom_column}")).radius * 2)"
           R"( WHERE width IS NULL)");

    if (!append_mode()) {
        dbexec("ANALYZE {qualified_src_areas}");
    }

    timer(m_timer_area).stop();

    log_gen("Get 'width' from areas onto lines...");
    timer(m_timer_prep).start();
    dbexec(R"(
WITH _covered_lines AS (
    SELECT "{geom_column}" AS geom, "{id_column}" AS wid FROM {src} w
        WHERE ST_NumPoints(w."{geom_column}") > 2 AND ST_CoveredBy(w."{geom_column}",
            (SELECT ST_Union("{geom_column}") FROM {qualified_src_areas} a
                WHERE ST_Intersects(w."{geom_column}", a."{geom_column}")))
), _intersections AS (
    SELECT w.wid, ST_Intersection(a.geom, w.geom) AS inters,
           ST_Length(w.geom) AS wlength, a.width AS width
        FROM _covered_lines w, {qualified_src_areas} a
        WHERE ST_Intersects(w.geom, a.geom)
), _lines AS (
    SELECT wid, wlength, ST_Length(inters) * width AS lenwidth FROM _intersections
        WHERE ST_GeometryType(inters) IN ('ST_LineString', 'ST_MultiLineString')
), _glines AS (
    SELECT wid, sum(lenwidth) / wlength AS width FROM _lines
    GROUP BY wid, wlength
)
UPDATE {src} a SET width = l.width
    FROM _glines l WHERE l.wid = a."{id_column}" AND a.width IS NULL
    )");
    timer(m_timer_prep).stop();

    log_gen("Reading waterway lines from database...");
    get_stats();

    // This vector will initially contain all segments (connection between
    // two points) from waterway ways. They will later be assembled into
    // graph edges connecting points where the waterways network branches.
    std::vector<edge_t> edges;
    edges.reserve(m_num_points - m_num_waterways);

    // This stores the order of each node in our graph, i.e. the number of
    // connections this node has. Order 1 are beginning or end of a waterway,
    // order 2 is just the continuing waterway, order >= 3 is a branching
    // point.
    std::map<geom::point_t, uint8_t> node_order;

    // This is where we keep the names of all waterways indexed by their
    // way id.
    std::unordered_map<osmid_t, std::string> names;

    timer(m_timer_get).start();
    {
        auto const result = dbexec(R"(
SELECT "{id_column}", "{width_column}", "{name_column}", "role", "{geom_column}"
 FROM {src};
)");

        for (int i = 0; i < result.num_tuples(); ++i) {
            auto const id = std::strtol(result.get_value(i, 0), nullptr, 10);
            auto const width = std::strtod(result.get_value(i, 1), nullptr);
            auto const name = result.get(i, 2);
            if (!name.empty()) {
                names.emplace(id, name);
            }
            auto const role = result.get(i, 3);
            auto const geom = ewkb_to_geom(util::decode_hex(result.get(i, 4)));

            if (geom.is_linestring()) {
                auto const &ls = geom.get<geom::linestring_t>();
                geom::for_each_segment(ls,
                                       [&](geom::point_t a, geom::point_t b) {
                                           if (a != b) {
                                               auto &f = edges.emplace_back();
                                               f.points.push_back(a);
                                               f.points.push_back(b);
                                               f.id = id;
                                               f.width = width;
                                               f.role = role;
                                               node_order[a]++;
                                               node_order[b]++;
                                           }
                                       });
            }
        }
    }
    timer(m_timer_get).stop();
    log_gen("Read {} segments, {} unique points, and {} names.", edges.size(),
            node_order.size(), names.size());

    if (edges.size() < 2) {
        log_gen("Found fewer than two segments. Nothing to do.");
        return;
    }

    log_gen("Sorting segments...");
    timer(m_timer_sort).start();
    std::sort(edges.begin(), edges.end());
    timer(m_timer_sort).stop();

    log_gen("Assembling edges from segments...");
    timer(m_timer_net).start();
    for (auto &edge : edges) {
        if (edge.points.size() > 1) {
            assemble_edge(&edge, &edges, node_order);
        }
    }
    timer(m_timer_net).stop();

    log_gen("Breaking cycles in river network...");
    timer(m_timer_break_cycles).start();
    break_cycles(edges);
    timer(m_timer_break_cycles).stop();

    log_gen("Removing now empty edges...");
    timer(m_timer_remove).start();
    {
        auto const last =
            std::remove_if(edges.begin(), edges.end(), [](edge_t const &edge) {
                return edge.points.size() == 1;
            });
        edges.erase(last, edges.end());
        std::sort(edges.begin(), edges.end());
    }
    timer(m_timer_remove).stop();

    log_gen("Network has {} edges.", edges.size());

    log_gen("Propagating 'width' property downstream...");
    timer(m_timer_width).start();
    for (auto &edge : edges) {
        assert(!edge.points.empty());
        geom::linestring_t seen;
        follow_chain_and_set_width(edge, &edges, node_order, &seen);
    }
    timer(m_timer_width).stop();

    log_gen("Calculating 'rank' property...");
    timer(m_timer_rank).start();
    {
        std::map<geom::point_t, std::vector<edge_t *>> end_point_map;
        for (auto &edge : edges) {
            end_point_map[edge.points.back()].push_back(&edge);
        }

        // Initialize source nodes with rank 1
        for (auto &edge : edges) {
            auto const parent_it = end_point_map.find(edge.points.front());
            if (parent_it == end_point_map.end()) {
                edge.rank = 1;
            }
        }

        bool changed_in_pass = true;
        int passes = 0;
        while (changed_in_pass) {
            changed_in_pass = false;
            passes++;
            log_debug("Rank calculation pass {}", passes);

            for (auto &current_edge : edges) {
                // Skip source nodes, their rank is already 1 and final
                auto const parent_it = end_point_map.find(current_edge.points.front());
                if (parent_it == end_point_map.end()) {
                    continue;
                }

                auto const &parents = parent_it->second;

                // Check if all parents have had their ranks calculated (rank > 0)
                bool all_parents_ranked = true;
                double sum_parent_ranks = 0;
                for (auto const *parent_edge : parents) {
                    if (parent_edge->rank == 0) { // Rank 0 means it's not yet calculated
                        all_parents_ranked = false;
                        break;
                    }
                    sum_parent_ranks += parent_edge->rank;
                }

                if (!all_parents_ranked) {
                    continue; // Can't calculate rank yet, will try in a later pass
                }

                // Find siblings. All parents should merge to the same point.
                // Assuming parents[0] is representative for the junction
                auto const *first_parent = parents[0];
                auto const [s, e] = std::equal_range(edges.begin(), edges.end(), first_parent->points.back());
                
                auto const num_children_at_fork = std::distance(s, e);

                double new_rank = 0;
                if (num_children_at_fork <= 1) {
                    // Rule 2: Single child
                    new_rank = sum_parent_ranks;
                } else {
                    // Rule 3: Multiple children
                    // New two-tiered logic to find the main sibling
                    std::vector<edge_t *> main_stream_siblings;
                    for (auto it = s; it != e; ++it) {
                        if (it->role == "main_stream") {
                            main_stream_siblings.push_back(&*it);
                        }
                    }

                    edge_t *max_width_sibling = nullptr;
                    if (!main_stream_siblings.empty()) {
                        // Tier 1: A main_stream role exists. Find the widest among them.
                        max_width_sibling = main_stream_siblings[0];
                        for (size_t i = 1; i < main_stream_siblings.size(); ++i) {
                            if (main_stream_siblings[i]->width > max_width_sibling->width) {
                                max_width_sibling = main_stream_siblings[i];
                            }
                        }
                    } else {
                        // Tier 2: No main_stream role. Fall back to widest among all children.
                        max_width_sibling = &*s;
                        for (auto it = std::next(s); it != e; ++it) {
                            if (it->width > max_width_sibling->width) {
                                max_width_sibling = &*it;
                            }
                        }
                    }

                    if (&current_edge == max_width_sibling) {
                        // Rule 3b, with cap at 1
                        new_rank = std::max(1.0, sum_parent_ranks - num_children_at_fork + 1);
                    } else {
                        // Rule 3a
                        new_rank = 1;
                    }
                }
                
                if (current_edge.rank != new_rank) {
                    current_edge.rank = new_rank;
                    changed_in_pass = true;
                }
            }
        }
        log_gen("Rank calculation completed in {} passes.", passes);
    }
    timer(m_timer_rank).stop();

    if (append_mode()) {
        dbexec("TRUNCATE {dest}");
    }

    log_gen("Writing results to destination table...");
    dbprepare("ins", "INSERT INTO {dest} ({id_column}, width, rank, name, role, geom)"
                     " VALUES ($1::int8, $2::real, $3::real, $4::text, $5::text, $6::geometry)");

    timer(m_timer_write).start();
    connection().exec("BEGIN");
    for (auto &edge : edges) {
        geom::geometry_t const geom{std::move(edge.points), PROJ_SPHERE_MERC};
        auto const wkb = geom_to_ewkb(geom);
        connection().exec_prepared("ins", edge.id, edge.width, edge.rank,
                                   get_name(names, edge.id), edge.role,
                                   binary_param_t(wkb));
    }
    connection().exec("COMMIT");
    timer(m_timer_write).stop();

    if (!append_mode()) {
        dbexec("ANALYZE {dest}");
    }

    log_gen("Done.");
}
