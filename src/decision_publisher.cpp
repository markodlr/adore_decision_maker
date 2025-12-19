/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/


#include "decision_publisher.hpp"

namespace adore
{

void
DecisionPublisher::setup( rclcpp::Node& node, const OutTopics& topics )
{
  trajectory_publisher            = node.create_publisher<TrajectoryAdapter>( topics.trajectory_decision, 1 );
  trajectory_suggestion_publisher = node.create_publisher<TrajectoryAdapter>( topics.trajectory_suggestion, 1 );
  assistance_publisher            = node.create_publisher<adore_ros2_msgs::msg::AssistanceRequest>( topics.assistance_request, 1 );
  traffic_participant_publisher   = node.create_publisher<ParticipantAdapter>( topics.traffic_participant, 1 );
  drivable_area_publisher         = node.create_publisher<DrivableAreaAdapter>( topics.drivable_area, 1 );
}

void
DecisionPublisher::publish( const rclcpp::Node& node, const Decision& decision )
{
  if( decision.drivable_area )
  {
    drivable_area_publisher->publish( *decision.drivable_area );
  }
  if( decision.trajectory )
    trajectory_publisher->publish( *decision.trajectory );

  if( decision.trajectory_suggestion )
    trajectory_suggestion_publisher->publish( *decision.trajectory_suggestion );

  if( decision.assistance_request )
  {
    adore_ros2_msgs::msg::AssistanceRequest assistance_request;
    assistance_request.assistance_needed = decision.assistance_request.value();
    assistance_request.state             = dynamics::conversions::to_ros_msg( decision.traffic_participant.value().state );
    assistance_request.header.stamp      = node.now();
    assistance_publisher->publish( assistance_request );
  }
  traffic_participant_publisher->publish( *decision.traffic_participant );
}


} // namespace adore