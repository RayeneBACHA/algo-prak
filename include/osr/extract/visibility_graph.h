#pragma once

#include "osr/extract/walkable_areas.h"

namespace osr {
bool is_visible(walkable_area const& area,
                geo::latlng const& start,
                geo::latlng const& end);

void build_visibility_graph(walkable_area const& area,
                            virtual_ways& vw);

}  // namespace osr
