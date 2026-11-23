#include "osr/extract/visibility_graph.h"
#include "osr/ways.h"

#include <array>
#include <memory>
#include <span>
#include <vector>
#include <algorithm>

#include "tg.h"

#include <optional>

namespace osr {
namespace {

using ring_ptr = std::unique_ptr<tg_ring, decltype(&tg_ring_free)>;
using poly_ptr = std::unique_ptr<tg_poly, decltype(&tg_poly_free)>;
using geom_ptr = std::unique_ptr<tg_geom, decltype(&tg_geom_free)>;
using line_ptr = std::unique_ptr<tg_line, decltype(&tg_line_free)>;

// 1. Convert OSR coordinates to a TG ring.
ring_ptr make_ring(std::vector<geo::latlng> const& coords) {
  auto points = std::vector<tg_point>{};
  points.reserve(coords.size());

  for (auto const& p : coords) {
    points.push_back(tg_point{p.lng(), p.lat()});
  }

  return {tg_ring_new(points.data(), static_cast<int>(points.size())),
          &tg_ring_free};
}

// 2. Create a polygon wiht optional holes.
geom_ptr make_polygon(
  std::vector<geo::latlng> const& outer_coords,
  std::span<area_ring const> hole_rings = {}
) {

  ring_ptr outer = make_ring(outer_coords);

  if (!outer) {
    return {nullptr, &tg_geom_free};
  }

  std::vector<ring_ptr> holes;
  std::vector<tg_ring const*> hole_ptrs;
  holes.reserve(hole_rings.size());
  hole_ptrs.reserve(hole_rings.size());

  for (auto const& hole : hole_rings) {
    holes.push_back(make_ring(hole.points_));

    if (!holes.back()) {
      return {nullptr,&tg_geom_free};
    }

    hole_ptrs.push_back(holes.back().get());
  }

  auto polygon = poly_ptr{
    tg_poly_new(
      outer.get(), hole_ptrs.data(),
      static_cast<int>(hole_ptrs.size())),
    &tg_poly_free
  };

  if (!polygon) {
    return {nullptr, &tg_geom_free};
  }

  return {tg_geom_new_polygon(polygon.get()), &tg_geom_free};  
}
// 3. Construct the walkable polygon.
geom_ptr make_area_polygon(walkable_area const& area) {
  if (area.rings_.empty()) {
    return {nullptr, &tg_geom_free};
  }

  return make_polygon(
    area.rings_.front().points_,
    std::span<area_ring const>{area.rings_}.subspan(1)
  );
  
}

// 4. Convert two OSR coordinates into a TG line geometry.
geom_ptr make_segment(
  geo::latlng const& start,
  geo::latlng const& end
) {
  std::array<tg_point, 2> endpoints = {{
    {start.lng(), start.lat()},
    {end.lng(), end.lat()}
  }};

  auto segment = line_ptr{
    tg_line_new(endpoints.data(), static_cast<int>(endpoints.size())),
    &tg_line_free
  };

  if (!segment) {
    return {nullptr, &tg_geom_free};
  }

  return {tg_geom_new_linestring(segment.get()), &tg_geom_free};
}

// 5. Check whether a segment intersects a building.
bool intersects_building(
  tg_geom const* segment,
  std::vector<geo::latlng> const& building
) {
  
  geom_ptr building_polygon = make_polygon(building);

  if (!building_polygon)
  {
    return true;
  }

  return tg_geom_intersects(segment, building_polygon.get()) &&
    !tg_geom_touches(segment, building_polygon.get());
  
}
// 6. Check whether a segment intersects a barrier.
bool intersects_barrier(
  tg_geom const* segment,
  std::array<geo::latlng, 2> const& barrier
) {
  
  geom_ptr barrier_segment = make_segment(barrier[0], barrier[1]);

  if (!barrier_segment) {
    return true;
  }

  return tg_geom_intersects(segment, barrier_segment.get()) &&
         !tg_geom_touches(segment, barrier_segment.get());

}

struct visibility_vertex {
  geo::latlng pos_;
  std::optional<osm_node_idx_t> node_;
};

// Collects vertices...
std::vector<visibility_vertex> collect_vertices(walkable_area const& area)
{
  auto vertices = std::vector<visibility_vertex>{};

  // Existing connectors
  for (auto const& connector : area.connectors_) {
    vertices.push_back({connector.pos_, connector.node_});
  }

  auto add_corner = [&](geo::latlng const& pos) {
    auto const exists = std::ranges::any_of(
      vertices, [&](visibility_vertex const& v) { return v.pos_ == pos; }
    );

    if (!exists) {
      vertices.push_back({pos, std::nullopt});
    }

  };

  // Outer boundary and hole corners
  for (auto const& ring : area.rings_) {
    for (auto const& pos : ring.points_)
    {
      add_corner(pos);
    }
  }

  for (auto const& building : area.buildings_) {
    for (auto const& pos : building)
    {
      add_corner(pos);
    }
  }
  for (auto const& barrier : area.barriers_) {
    for (auto const& pos : barrier)
    {
      add_corner(pos);
    }
  }

  return vertices;
}

}  // namespace

// 7. Make the visibility decision.
bool is_visible(
  walkable_area const& area,
  geo::latlng const& start,
  geo::latlng const& end) {
    
    auto walkable_polygon = make_area_polygon(area);
    auto segment = make_segment(start, end);

    if (!walkable_polygon || !segment) {
      return false;
    }

    if (!tg_geom_covers(walkable_polygon.get(), segment.get())) {
      return false;
    }

    for (auto const& building : area.buildings_) {
      if (intersects_building(segment.get(), building)) {
        return false;
      }
    }

    for (auto const& barrier : area.barriers_) {
      if (intersects_barrier(segment.get(), barrier)) {
        return false;
      }
    }

    return true;
}

void build_visibility_graph(
  walkable_area const& area,
  virtual_ways& vw
) {
  if (area.connectors_.size() < 2U || area.rings_.empty()) {
    return;
  }

  auto vertices = collect_vertices(area);

  auto get_node = [&](visibility_vertex& vertex) -> osm_node_idx_t {
    if (!vertex.node_) {
      vertex.node_ = vw.add_node(vertex.pos_);
    }

    return *vertex.node_;
  };

  // Check every unordered pair of vertices.
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    for (std::size_t j = i + 1; j < vertices.size(); j++) {
      auto& a = vertices[i];
      auto& b = vertices[j];

      // Avoid zero_length edges.
      if (a.pos_ == b.pos_){
        continue;
      }

      if (!is_visible(area, a.pos_, b.pos_)) {
        continue;
      }

      std::array<osm_node_idx_t, 2> nodes = {
        get_node(a),
        get_node(b)
      };

      std::array<geo::latlng, 2> polyline = {
        a.pos_,
        b.pos_
      };

      vw.add_way(
        virtual_ways::foot_way(kNoLevel),
        nodes,
        polyline
      );
    }
  }
}

}  // namespace osr