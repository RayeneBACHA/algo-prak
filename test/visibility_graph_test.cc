#include "osr/extract/visibility_graph.h"
#include "osr/extract/detail/visibility_geometry.h"
#include <memory>
#include "gtest/gtest.h"
#include "tg.h"

namespace osr {

namespace {

using geom_ptr = std::unique_ptr<tg_geom, decltype(&tg_geom_free)>;

geom_ptr parse_geometry(char const* wkt) {
  return {tg_parse_wkt(wkt), &tg_geom_free};
}

walkable_area make_barrier_test_area() {
  walkable_area area{};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {
      {49.000, 8.000},
      {49.000, 8.010},
      {49.010, 8.010},
      {49.010, 8.000},
      {49.000, 8.000}
  };

  area.barriers_.push_back({
      geo::latlng{49.002, 8.005},
      geo::latlng{49.008, 8.005}
  });

  return area;
}

// Test-only helper. Not part of the public OSR API.
bool is_visible(
  walkable_area const& area,
  geo::latlng const& start,
  geo::latlng const& end) {

    auto const geometry = detail::visibility_geometry{area};

    return detail::is_visible_cached(
      geometry, start, end
    );
  } 

}  // namespace

TEST(visibility_geometry, line_inside_rectangle) {

  auto const area = parse_geometry("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");

  auto const line = parse_geometry("LINESTRING(2 2, 8 8)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));

  EXPECT_TRUE(tg_geom_covers(area.get(), line.get()));
}

TEST(visibility_geometry, line_outside_rectangle) {

  auto const area = parse_geometry("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");

  auto const line = parse_geometry("LINESTRING(2 2, 12 8)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));

  EXPECT_FALSE(tg_geom_covers(area.get(), line.get()));
}

TEST(visibility_geometry, line_avoids_hole) {
  auto const area = parse_geometry(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0),"
      "(4 4, 4 6, 6 6, 6 4, 4 4))");

  auto const line = parse_geometry("LINESTRING(2 2, 8 2)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));

  EXPECT_TRUE(tg_geom_covers(area.get(), line.get()));
}

TEST(visibility_geometry, line_crosses_hole) {
  auto const area = parse_geometry(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0),"
      "(4 4, 4 6, 6 6, 6 4, 4 4))");

  auto const line = parse_geometry("LINESTRING(2 5, 8 5)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));

  EXPECT_FALSE(tg_geom_covers(area.get(), line.get()));
}

TEST(visibility_geometry, line_inside_concave_polygon) {
  auto const area =
      parse_geometry("POLYGON((0 0, 10 0, 10 4, 4 4, 4 10, 0 10, 0 0))");

  auto const line = parse_geometry("LINESTRING(1 2, 1 8)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));

  EXPECT_TRUE(tg_geom_covers(area.get(), line.get()));
}

TEST(visibility_geometry, line_exits_concave_polygon) {
  auto const area =
      parse_geometry("POLYGON((0 0, 10 0, 10 4, 4 4, 4 10, 0 10, 0 0))");

  auto const line = parse_geometry("LINESTRING(2 8, 8 2)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));

  EXPECT_FALSE(tg_geom_covers(area.get(), line.get()));
}

TEST(visibility_geometry, line_crosses_building) {
  auto const area = parse_geometry("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");

  auto const building = parse_geometry("POLYGON((4 4, 6 4, 6 6, 4 6, 4 4))");

  auto const line = parse_geometry("LINESTRING(2 5, 8 5)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());
  ASSERT_NE(nullptr, building.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));
  ASSERT_EQ(nullptr, tg_geom_error(building.get()));

  EXPECT_TRUE(tg_geom_covers(area.get(), line.get()));
  EXPECT_TRUE(tg_geom_intersects(line.get(), building.get()));
}

TEST(visibility_geometry, line_avoids_building) {
  auto const area = parse_geometry("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");

  auto const building = parse_geometry("POLYGON((4 4, 6 4, 6 6, 4 6, 4 4))");

  auto const line = parse_geometry("LINESTRING(2 2, 8 2)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());
  ASSERT_NE(nullptr, building.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));
  ASSERT_EQ(nullptr, tg_geom_error(building.get()));

  EXPECT_TRUE(tg_geom_covers(area.get(), line.get()));
  EXPECT_FALSE(tg_geom_intersects(line.get(), building.get()));
}
TEST(visibility_geometry, line_touches_building_corner) {
  auto const area = parse_geometry("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");

  auto const building = parse_geometry("POLYGON((4 4, 6 4, 6 6, 4 6, 4 4))");

  auto const line = parse_geometry("LINESTRING(2 2, 4 4)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());
  ASSERT_NE(nullptr, building.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));
  ASSERT_EQ(nullptr, tg_geom_error(building.get()));

  EXPECT_TRUE(tg_geom_covers(area.get(), line.get()));
  EXPECT_TRUE(tg_geom_intersects(line.get(), building.get()));
}

TEST(visibility_geometry, line_crosses_barrier) {
  auto const area = parse_geometry("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");

  auto const barrier = parse_geometry("LINESTRING(5 2, 5 8)");

  auto const line = parse_geometry("LINESTRING(2 5, 8 5)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());
  ASSERT_NE(nullptr, barrier.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));
  ASSERT_EQ(nullptr, tg_geom_error(barrier.get()));

  EXPECT_TRUE(tg_geom_covers(area.get(), line.get()));
  EXPECT_TRUE(tg_geom_intersects(line.get(), barrier.get()));
}

TEST(visibility_geometry, line_avoids_barrier) {
  auto const area = parse_geometry("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");

  auto const barrier = parse_geometry("LINESTRING(5 2, 5 8)");

  auto const line = parse_geometry("LINESTRING(2 1, 8 1)");

  ASSERT_NE(nullptr, area.get());
  ASSERT_NE(nullptr, line.get());
  ASSERT_NE(nullptr, barrier.get());

  ASSERT_EQ(nullptr, tg_geom_error(area.get()));
  ASSERT_EQ(nullptr, tg_geom_error(line.get()));
  ASSERT_EQ(nullptr, tg_geom_error(barrier.get()));

  EXPECT_TRUE(tg_geom_covers(area.get(), line.get()));
  EXPECT_FALSE(tg_geom_intersects(line.get(), barrier.get()));
}

TEST(visibility_geometry, visible_inside_area) {
  auto area = walkable_area{};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {{49.000, 8.000},
                                {49.000, 8.010},
                                {49.010, 8.010},
                                {49.010, 8.000},
                                {49.000, 8.000}};

  auto const start = geo::latlng{49.002, 8.002};
  auto const end = geo::latlng{49.008, 8.008};

  EXPECT_TRUE(is_visible(area, start, end));
}

TEST(visibility_geometry, invisible_outside_area) {
  auto area = walkable_area{};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {{49.000, 8.000},
                                {49.000, 8.010},
                                {49.010, 8.010},
                                {49.010, 8.000},
                                {49.000, 8.000}};

  auto const start = geo::latlng{49.002, 8.002};
  auto const end = geo::latlng{49.012, 8.008};

  EXPECT_FALSE(is_visible(area, start, end));
}

TEST(visibility_geometry, invisible_through_hole) {
  auto area = walkable_area{};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {{49.000, 8.000},
                                {49.000, 8.010},
                                {49.010, 8.010},
                                {49.010, 8.000},
                                {49.000, 8.000}};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {{49.004, 8.004},
                                {49.006, 8.004},
                                {49.006, 8.006},
                                {49.004, 8.006},
                                {49.004, 8.004}};

  auto const start = geo::latlng{49.005, 8.002};
  auto const end = geo::latlng{49.005, 8.008};

  EXPECT_FALSE(is_visible(area, start, end));
}

TEST(visibility_geometry, visible_around_hole) {
  auto area = walkable_area{};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {{49.000, 8.000},
                                {49.000, 8.010},
                                {49.010, 8.010},
                                {49.010, 8.000},
                                {49.000, 8.000}};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {{49.004, 8.004},
                                {49.006, 8.004},
                                {49.006, 8.006},
                                {49.004, 8.006},
                                {49.004, 8.004}};

  auto const start = geo::latlng{49.002, 8.002};
  auto const end = geo::latlng{49.002, 8.008};

  EXPECT_TRUE(is_visible(area, start, end));
}

TEST(visibility_geometry, invisible_through_building) {
  auto area = walkable_area{};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {{49.000, 8.000},
                                {49.000, 8.010},
                                {49.010, 8.010},
                                {49.010, 8.000},
                                {49.000, 8.000}};

  area.buildings_.push_back({{49.004, 8.004},
                             {49.004, 8.006},
                             {49.006, 8.006},
                             {49.006, 8.004},
                             {49.004, 8.004}});

  auto const start = geo::latlng{49.005, 8.002};
  auto const end = geo::latlng{49.005, 8.008};

  EXPECT_FALSE(is_visible(area, start, end));
}

TEST(visibility_geometry, invisible_through_barrier) {
  auto area = walkable_area{};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {
    {49.000, 8.000},
    {49.000, 8.010},
    {49.010, 8.010},
    {49.010, 8.000},
    {49.000, 8.000}
  };

  area.barriers_.push_back({
    geo::latlng{49.003, 8.005},
    geo::latlng{49.007, 8.005}
  });

  auto const start = geo::latlng{49.005, 8.002};
  auto const end = geo::latlng{49.005, 8.008};

  EXPECT_FALSE(is_visible(area, start, end));

}


TEST(visibility_geometry, visible_at_building_corner) {
  auto area = walkable_area{};

  area.rings_.emplace_back();
  area.rings_.back().points_ = {{49.000, 8.000},
                                {49.000, 8.010},
                                {49.010, 8.010},
                                {49.010, 8.000},
                                {49.000, 8.000}};

  area.buildings_.push_back({{49.004, 8.004},
                             {49.004, 8.006},
                             {49.006, 8.006},
                             {49.006, 8.004},
                             {49.004, 8.004}});

  auto const start = geo::latlng{49.002, 8.002};
  auto const end = geo::latlng{49.004, 8.004};

  EXPECT_TRUE(is_visible(area, start, end));
}

TEST(visibility_geometry, visible_at_barrier_endpoint) {
  auto area = make_barrier_test_area();

  auto const start = geo::latlng{49.002, 8.002};
  auto const end = geo::latlng{49.002, 8.005};

  EXPECT_TRUE(is_visible(area, start, end));

}

TEST(visibility_geometry, invisible_along_barrier) {
  auto area = make_barrier_test_area();

  auto const start = geo::latlng{49.003, 8.005};
  auto const end = geo::latlng{49.005, 8.005};

  EXPECT_FALSE(is_visible(area, start, end));
}


}  // namespace osr
