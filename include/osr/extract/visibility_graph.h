#pragma once

#include "osr/extract/walkable_areas.h"

namespace osr {

void build_visibility_graph(walkable_area const& area,
                            virtual_ways& vw);

}  // namespace osr
