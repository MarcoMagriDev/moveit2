#pragma once
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace moveit_servo
{

class ArrowMarkerPublisher
{
public:
  ArrowMarkerPublisher(rclcpp::Node::SharedPtr node)
  {
    marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("arrow_markers", 10);
  }
  void add_marker(collision_detection::Contact c)
  {
    visualization_msgs::msg::Marker marker_msg_;
    marker_msg_.header.frame_id = "panda_link0";  // Replace with your frame ID
    marker_msg_.id = marker_array_msg_.markers.size();
    marker_msg_.type = visualization_msgs::msg::Marker::ARROW;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;
    marker_msg_.pose.orientation.w = 1.0;
    marker_msg_.scale.x = 0.01;  // Arrow shaft diameter
    marker_msg_.scale.y = 0.02;  // Arrow head diameter
    marker_msg_.scale.z = 0.0;
    marker_msg_.color.a = 1.0;  // Alpha (transparency)
    marker_msg_.color.r = 1.0;  // Red
    marker_msg_.color.g = 0.0;  // Green
    marker_msg_.color.b = 0.0;  // Blue

    geometry_msgs::msg::Point start_point;
    start_point.x = c.nearest_points[0][0];
    start_point.y = c.nearest_points[0][1];
    start_point.z = c.nearest_points[0][2];

    geometry_msgs::msg::Point end_point;
    end_point.x = c.nearest_points[1][0];
    end_point.y = c.nearest_points[1][1];
    end_point.z = c.nearest_points[1][2];

    marker_msg_.points.push_back(start_point);
    marker_msg_.points.push_back(end_point);
    marker_array_msg_.markers.push_back(marker_msg_);

    // Add Sphere Marker
    visualization_msgs::msg::Marker sphere_marker_msg1;
    sphere_marker_msg1.header.frame_id = "panda_link0";  // Replace with your frame ID
    sphere_marker_msg1.id = marker_array_msg_.markers.size();
    sphere_marker_msg1.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker_msg1.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker_msg1.pose.position = start_point;  // Place the sphere at the end point of the arrow
    sphere_marker_msg1.scale.x = 0.025;              // Sphere diameter
    sphere_marker_msg1.scale.y = 0.025;
    sphere_marker_msg1.scale.z = 0.025;
    sphere_marker_msg1.color.a = 1.0;  // Alpha (transparency)
    sphere_marker_msg1.color.r = 0.0;  // Red
    sphere_marker_msg1.color.g = 0.0;  // Green
    sphere_marker_msg1.color.b = 1.0;  // Blue
    marker_array_msg_.markers.push_back(sphere_marker_msg1);

    // Add Sphere Marker
    visualization_msgs::msg::Marker sphere_marker_msg2;
    sphere_marker_msg2.header.frame_id = "panda_link0";  // Replace with your frame ID
    sphere_marker_msg2.id = marker_array_msg_.markers.size();
    sphere_marker_msg2.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker_msg2.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker_msg2.pose.position = end_point;  // Place the sphere at the end point of the arrow
    sphere_marker_msg2.scale.x = 0.025;            // Sphere diameter
    sphere_marker_msg2.scale.y = 0.025;
    sphere_marker_msg2.scale.z = 0.025;
    sphere_marker_msg2.color.a = 1.0;  // Alpha (transparency)
    sphere_marker_msg2.color.r = 0.0;  // Red
    sphere_marker_msg2.color.g = 0.0;  // Green
    sphere_marker_msg2.color.b = 1.0;  // Blue
    marker_array_msg_.markers.push_back(sphere_marker_msg2);
  }
  void publish()
  {
    if (!last_marker_array_msg_.markers.empty())
    {
      for (auto& marker : last_marker_array_msg_.markers)
      {
        marker.action = visualization_msgs::msg::Marker::DELETE;
      }
      marker_publisher_->publish(last_marker_array_msg_);
    }
    marker_publisher_->publish(marker_array_msg_);
    last_marker_array_msg_ = marker_array_msg_;
    marker_array_msg_.markers.clear();
  }

private:
  visualization_msgs::msg::MarkerArray marker_array_msg_;
  visualization_msgs::msg::MarkerArray last_marker_array_msg_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

class DirectionalCollisionChecker
{
public:
  DirectionalCollisionChecker(const rclcpp::Node::SharedPtr node, const ServoParameters::SharedConstPtr& parameters,
                              const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
    : parameters_{ parameters }, planning_scene_monitor_(planning_scene_monitor)
  {
    arrow_marker_publisher_ = std::make_shared<ArrowMarkerPublisher>(node);
    request.group_name = parameters_->move_group_name;
    request.contacts = true;
    request.distance = true;
    // This may be limiting: if one single object with corner is present only the closest point is captured
    // request.max_contacts_per_pair = 1;
  };

  void update_delta_x(Eigen::VectorXd& delta_x)
  {
    auto start = std::chrono::high_resolution_clock::now();

    bool collision_detected{ false };
    auto scene = planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_);
    // TODO: is this taking too long? is there any solution to avoid this each update cycle?
    scene->allocateCollisionDetector(bullet_allocator_);

    auto state = scene->getCurrentState();
    state.updateLinkTransforms();
    state.updateCollisionBodyTransforms();

    auto result = collision_detection::CollisionResult();
    scene->getCollisionEnv()->checkRobotCollision(request, result, state);
    collision_detected |= result.collision;
    scene_contacts = result.contacts;

    // TODO: acm seems to be not considered
    scene->getCollisionEnvUnpadded()->checkSelfCollision(request, result, state, scene->getAllowedCollisionMatrix());
    collision_detected |= result.collision;
    self_contacts = result.contacts;

    if (collision_detected)
    {
      delta_x.setZero();
      return;
    }

    // Loop trough scene collision and apply directional scaling for each collision
    for (const auto& [contact_pairs, collisions] : scene_contacts)
    {
      for (auto& c : collisions)
      {
        apply_contribution(delta_x, c, parameters_->scene_collision_proximity_threshold, state);
      }
    }
    // Loop trough self collisions and apply directional scaling for each collision
    for (const auto& [contact_pairs, collisions] : self_contacts)
    {
      for (auto& c : collisions)
      {
        apply_contribution(delta_x, c, parameters_->self_collision_proximity_threshold, state);
      }
    }
    arrow_marker_publisher_->publish();
    std::cout
        << "Time taken by function: "
        << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start)
               .count()
        << " microseconds" << std::endl;
  };

  void apply_contribution(Eigen::VectorXd& delta_x, const collision_detection::Contact& contact,
                          const double& proximity_threshold, const moveit::core::RobotState& state)
  {
    Eigen::Vector3d contact_vect = (contact.nearest_points[1] - contact.nearest_points[0]);
    double distance = contact_vect.norm();
    if (distance > proximity_threshold)
    {
      // noting to be scaled
      return;
    }
    arrow_marker_publisher_->add_marker(contact);

    // Translational scaling
    Eigen::Vector3d contribution = contact_vect / distance;
    // distance = exp(-log(0.001) / proximity_threshold * (distance - proximity_threshold));
    double scaling = contribution.dot(delta_x.head(3) / delta_x.head(3).norm());
    if (scaling > 0)
    {
      delta_x.head(3) = delta_x.head(3) - delta_x.head(3).norm() * scaling * contribution /* * (scaling) */;
    }

    // Rotational scaling: rotations contacts will be avoided by translational motions
    // TODO: handle corner cases in which this compensation would cause the collision due to additional velocity introduced
    // A solution may be inverting the two but then the problem may be the opposite:
    // not enough translational motion is performed to avoid collisions due to rotation.
    // The solution may be to scale the rotational component instead of just adding translation velocity
    auto r = contact.nearest_points[0] - state.getGlobalLinkTransform(contact.body_name_1).translation();
    Eigen::Vector3d w = delta_x.tail(3);
    Eigen::Vector3d v = w.cross(r);
    scaling = contribution.dot(v / v.norm());
    if (scaling > 0)
    {
      Eigen::Vector3d v_scaled = v - v.norm() * scaling * contribution;
      // TODO: add w scaling if possibile
      delta_x.head(3) += v_scaled - v;  //;
      // Minus removes the rotable part of the velocity_scaled.
      // v_rot = v_scaled - v_scaled.dot(v / v.norm()) * v / v.norm();
    }
  }

private:
  ServoParameters::SharedConstPtr parameters_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  collision_detection::CollisionRequest request = collision_detection::CollisionRequest();
  std::shared_ptr<ArrowMarkerPublisher> arrow_marker_publisher_;
  std::shared_ptr<collision_detection::CollisionDetectorAllocator> bullet_allocator_ =
      collision_detection::CollisionDetectorAllocatorBullet::create();
  collision_detection::CollisionResult::ContactMap scene_contacts, self_contacts;
};

}  // namespace moveit_servo