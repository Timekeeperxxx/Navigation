#include <global_planner/a_star_on_pc.h>

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

namespace
{

pcl::PointCloud<pcl::PointXYZI>::Ptr makeCloud(std::size_t size)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
    new pcl::PointCloud<pcl::PointXYZI>());
  cloud->resize(size);
  for (std::size_t i = 0; i < size; ++i) {
    cloud->points[i].x = static_cast<float>(i);
  }
  return cloud;
}

Node_t makeOpenNode(unsigned int index, float g, float f)
{
  Node_t node;
  node.self_index = index;
  node.g = g;
  node.h = f - g;
  node.f = f;
  node.parent_index = index;
  node.is_opened = true;
  return node;
}

TEST(AstarList, EmptyFrontierNeverReturnsNodeZero)
{
  auto cloud = makeCloud(2);
  AstarList list(cloud);
  list.Initial();

  Node_t popped;
  EXPECT_FALSE(list.tryPopNodeWithMinimumF(popped));
  EXPECT_EQ(popped.self_index, std::numeric_limits<unsigned int>::max());
  EXPECT_EQ(popped.parent_index, std::numeric_limits<unsigned int>::max());

  const Node_t compatibility_result = list.getNode_wi_MinimumF();
  EXPECT_EQ(
    compatibility_result.self_index,
    std::numeric_limits<unsigned int>::max());
  EXPECT_EQ(
    compatibility_result.parent_index,
    std::numeric_limits<unsigned int>::max());
}

TEST(AstarList, PriorityQueueDropsSupersededEntries)
{
  auto cloud = makeCloud(3);
  AstarList list(cloud);
  list.Initial();

  Node_t old_node = makeOpenNode(0, 9.0f, 10.0f);
  Node_t other_node = makeOpenNode(1, 6.0f, 7.0f);
  Node_t improved_node = makeOpenNode(0, 4.0f, 5.0f);
  list.updateNode(old_node);
  list.updateNode(other_node);
  list.updateNode(improved_node);

  Node_t popped;
  ASSERT_TRUE(list.tryPopNodeWithMinimumF(popped));
  EXPECT_EQ(popped.self_index, 0U);
  EXPECT_FLOAT_EQ(popped.g, 4.0f);

  list.closeNode(popped);
  ASSERT_TRUE(list.tryPopNodeWithMinimumF(popped));
  EXPECT_EQ(popped.self_index, 1U);
  list.closeNode(popped);

  // The old queue entry for node 0 is still physically present, but its
  // revision is stale and must be drained without fabricating a node.
  EXPECT_TRUE(list.isFrontierEmpty());
  EXPECT_FALSE(list.tryPopNodeWithMinimumF(popped));
  EXPECT_EQ(popped.self_index, std::numeric_limits<unsigned int>::max());
}

TEST(AstarList, ClosedOnlyFrontierEndsCleanly)
{
  auto cloud = makeCloud(1);
  AstarList list(cloud);
  list.Initial();

  Node_t node = makeOpenNode(0, 0.0f, 1.0f);
  list.updateNode(node);
  list.closeNode(node);

  EXPECT_TRUE(list.isFrontierEmpty());
  Node_t popped;
  EXPECT_FALSE(list.tryPopNodeWithMinimumF(popped));
  EXPECT_EQ(popped.self_index, std::numeric_limits<unsigned int>::max());
  EXPECT_EQ(popped.parent_index, std::numeric_limits<unsigned int>::max());
}

TEST(AStarPublicApi, OptionalPlanningControlsRemainCallable)
{
  using ExpectedEdgeValidator = std::function<bool(
    const pcl::PointXYZI&, const pcl::PointXYZI&)>;
  static_assert(std::is_same_v<
    A_Star_on_Graph::EdgeValidator, ExpectedEdgeValidator>);

  EXPECT_NE(&A_Star_on_Graph::setMaxPlanningTime, nullptr);
  EXPECT_NE(&A_Star_on_Graph::setCancelChecker, nullptr);
  EXPECT_NE(&A_Star_on_Graph::setEdgeValidator, nullptr);
  EXPECT_NE(&A_Star_on_Graph::setIndexEdgeValidator, nullptr);
  EXPECT_NE(&A_Star_on_Graph::setHeuristicWeight, nullptr);
  EXPECT_NE(&A_Star_on_Graph::setUsePerceptionCosts, nullptr);
  EXPECT_NE(&A_Star_on_Graph::wasTimedOut, nullptr);
}

}  // namespace
