#pragma once

#include <memory>
#include<vector>

#include "osr/extract/walkable_areas.h"
#include "tg.h"


namespace osr::detail {

using geom_ptr = 
    std::unique_ptr<tg_geom, decltype(&tg_geom_free)>;

struct visibility_geometry {
  explicit visibility_geometry(walkable_area const& area);

  geom_ptr polygon_;
  std::vector<geom_ptr> buildings_;
  std::vector<geom_ptr> barriers_;
  bool valid_{false};
};

bool is_visible_cached(
  visibility_geometry const& geometry,
  geo::latlng const& start,
  geo::latlng const& end);

} // namespace osr::detail