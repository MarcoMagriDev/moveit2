/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#pragma once

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit_msgs/msg/cost_source.hpp>
#include <moveit_msgs/msg/contact_information.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

namespace collision_detection
{
void getCollisionMarkersFromContacts(visualization_msgs::msg::MarkerArray& arr, const std::string& frame_id,
                                     const CollisionResult::ContactMap& con, const std_msgs::msg::ColorRGBA& color,
                                     const rclcpp::Duration& lifetime, const double radius = 0.035);

void getCollisionMarkersFromContacts(visualization_msgs::msg::MarkerArray& arr, const std::string& frame_id,
                                     const CollisionResult::ContactMap& con);

/// \todo add a class for managing cost sources
void getCostMarkers(visualization_msgs::msg::MarkerArray& arr, const std::string& frame_id,
                    std::set<CostSource>& cost_sources);

void getCostMarkers(visualization_msgs::msg::MarkerArray& arr, const std::string& frame_id,
                    std::set<CostSource>& cost_sources, const std_msgs::msg::ColorRGBA& color,
                    const rclcpp::Duration& lifetime);

double getTotalCost(const std::set<CostSource>& cost_sources);

void removeCostSources(std::set<CostSource>& cost_sources, const std::set<CostSource>& cost_sources_to_remove,
                       double overlap_fraction);
void intersectCostSources(std::set<CostSource>& cost_sources, const std::set<CostSource>& a,
                          const std::set<CostSource>& b);
void removeOverlapping(std::set<CostSource>& cost_sources, double overlap_fraction);

bool getSensorPositioning(geometry_msgs::msg::Point& point, const std::set<CostSource>& cost_sources);

void costSourceToMsg(const CostSource& cost_source, moveit_msgs::msg::CostSource& msg);
void contactToMsg(const Contact& contact, moveit_msgs::msg::ContactInformation& msg);
}  // namespace collision_detection
