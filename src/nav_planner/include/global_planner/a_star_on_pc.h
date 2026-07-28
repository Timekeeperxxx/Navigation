/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef A_STAR_ON_PC_H
#define A_STAR_ON_PC_H

/*For graph*/
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

/*For pcl::PointXYZ*/
#include <pcl/common/geometry.h>
#include <math.h>

/*
for graph
type edge_t is defined here
type graph_t is defined here
*/

/*For perception*/
#include <perception_3d/perception_3d_ros.h>
#include <global_planner/nanoflann_pcl.hpp>

typedef struct {
  unsigned int self_index = std::numeric_limits<unsigned int>::max();
  float g = std::numeric_limits<float>::infinity();
  float h = 0.0f;
  float f = std::numeric_limits<float>::infinity();
  unsigned int parent_index = std::numeric_limits<unsigned int>::max();
  bool is_closed = false;
  bool is_opened = false;
  //@ Track consecutive ground steps for hybrid planning
  //@ This allows the planner to penalize long sequences on ground points
  //@ and encourage returning to planground as soon as possible
  unsigned int consecutive_ground_steps = 0;
} Node_t;

typedef std::pair<double, unsigned int> f_p_;

class AstarList{
  public:
    AstarList(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up);

    void Initial();
    void updateNode(Node_t& a_node);
    void closeNode(Node_t& a_node);
    float getGVal(Node_t& a_node);
    bool tryPopNodeWithMinimumF(Node_t& node);
    Node_t getNode_wi_MinimumF();
    Node_t getNode(unsigned int node_index);
    bool isClosed(unsigned int node_index);
    bool isOpened(unsigned int node_index);
    bool isFrontierEmpty();
    /*Static graph is for path planning and list*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up_;
    nanoflann::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground_;
  private:
    struct FrontierEntry {
      float f;
      unsigned int node_index;
      std::uint64_t revision;
    };

    struct FrontierEntryGreater {
      bool operator()(const FrontierEntry& lhs, const FrontierEntry& rhs) const {
        if (lhs.f != rhs.f) {
          return lhs.f > rhs.f;
        }
        if (lhs.node_index != rhs.node_index) {
          return lhs.node_index > rhs.node_index;
        }
        return lhs.revision > rhs.revision;
      }
    };

    // Planning-cloud node IDs are the contiguous range [0, cloud.size()).
    // A vector avoids hash allocations and guarantees bounds-checked access.
    std::vector<Node_t> as_list_;
    std::vector<std::uint64_t> node_revisions_;
    std::priority_queue<
      FrontierEntry, std::vector<FrontierEntry>, FrontierEntryGreater>
      f_priority_queue_;
};

class A_Star_on_Graph{

    public:
      using CancelChecker = std::function<bool()>;
      using EdgeValidator = std::function<bool(
        const pcl::PointXYZI&, const pcl::PointXYZI&)>;
      using IndexEdgeValidator = std::function<bool(
        unsigned int, unsigned int)>;

      A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up, 
        std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
        double a_star_expanding_radius);
      
      ~A_Star_on_Graph();
      
      void updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up);

      void getPath( unsigned int start, unsigned int goal, std::vector<unsigned int>& path);
      
      void setupTurningWeight(double m_weight){turning_weight_ = m_weight;}
      void setMaxPlanningTime(double seconds);
      void setCancelChecker(CancelChecker checker);
      void setEdgeValidator(EdgeValidator validator);
      void setIndexEdgeValidator(IndexEdgeValidator validator);
      void setHeuristicWeight(double weight);
      /**
       * @brief Enable obstacle/dGraph costs while expanding the graph.
       *
       * The fill_footprint reference pass only needs a connectivity route. Its
       * returned polyline is checked afterwards by the full ground-footprint
       * validator, so building a perception lookup cache for every reference
       * node only delays (especially disconnected) searches.
       */
      void setUsePerceptionCosts(bool enabled);
      bool wasTimedOut() const { return planning_timed_out_; }
      bool wasCancelled() const { return planning_cancelled_; }

    private:
      struct PlanningNodeCache {
        unsigned int perception_ground_index =
          std::numeric_limits<unsigned int>::max();
        double dgraph_value = 0.0;
        float node_weight = 0.0f;
        bool initialized = false;
        bool valid = false;
      };

      
      //@ kd-tree for line-of-sight
      nanoflann::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_lethal_;

      //@ kd-tree for ground connectivity check (line-of-sight on ground)
      nanoflann::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground_los_;

      //@ Stable copy of perception ground used to translate arbitrary planning-cloud
      //@ indices (hybrid/planground) back to the dGraph/static-graph index space.
      pcl::PointCloud<pcl::PointXYZI>::Ptr perception_ground_cloud_;
      nanoflann::KdTreeFLANN<pcl::PointXYZI>::Ptr perception_ground_kdtree_;

      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up_;

      /*Provide dynamic graph for obstacle avoidance*/
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros_;
      
      /*Create the list*/
      AstarList* ASLS_;

      //@ turning weight of the node
      double turning_weight_;
      
      //@ neighborhodd expanding radius
      double a_star_expanding_radius_;

      double max_planning_time_seconds_ = 0.0;
      double heuristic_weight_ = 1.0;
      bool use_perception_costs_ = true;
      bool perception_cache_ready_ = false;
      std::vector<PlanningNodeCache> planning_node_cache_;
      CancelChecker cancel_checker_;
      EdgeValidator edge_validator_;
      IndexEdgeValidator index_edge_validator_;
      bool planning_timed_out_ = false;
      bool planning_cancelled_ = false;

      double getThetaFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding);
      double getPitchFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding);

      bool isLineOfSightClear(
        const pcl::PointXYZI& pcl_current,
        const pcl::PointXYZI& pcl_expanding,
        double inscribed_radius);
      bool getPerceptionGroundIndex(unsigned int planning_index, unsigned int& ground_index);
};

#endif // A_STAR_ON_PC_H
