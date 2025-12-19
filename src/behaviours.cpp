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

#include "behaviours.hpp"

#include "planning/drivable_area.hpp"
#include "planning/nominal_participant_prediction.hpp"
#include "planning/planning_helpers.hpp" // your existing helpers
#include "planning/speed_profile.hpp"

namespace adore::behaviours
{
Decision
emergency_stop( const Domain& domain, PlanningParams& planning_tools )
{

  Decision             out;
  dynamics::Trajectory emergency_stop_trajectory;
  if( domain.vehicle_state )
    emergency_stop_trajectory.states.push_back( domain.vehicle_state.value() );
  emergency_stop_trajectory.label = "Emergency Stop";
  out.trajectory                  = std::move( emergency_stop_trajectory );
  out.traffic_participant         = make_default_participant( domain, planning_tools );
  return out;
}

Decision
standstill( const Domain& domain, PlanningParams& planning_tools )
{

  Decision             out;
  dynamics::Trajectory standstill_trajectory;
  standstill_trajectory.label = "Standstill";
  if( domain.vehicle_state )
    standstill_trajectory.states.push_back( domain.vehicle_state.value() );
  out.trajectory          = std::move( standstill_trajectory );
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
follow_reference( const Domain& domain, PlanningParams& planning_tools )
{
  Decision out;
  out.trajectory        = *domain.reference_trajectory;
  out.trajectory->label = "Follow Reference";

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
follow_route( const Domain& domain, PlanningParams& planning_tools )
{
  Decision out;

  if( !domain.route || !domain.vehicle_state )
  {
    dynamics::Trajectory fallback;
    fallback.label          = "Follow Route (Invalid Domain)";
    out.trajectory          = std::move( fallback );
    out.traffic_participant = make_default_participant( domain, planning_tools );
    return out;
  }

  const auto& route = *domain.route;
  const auto& ego   = *domain.vehicle_state;

  planner::DrivableAreaConfig da_cfg;
  da_cfg.lane_scope             = planner::LaneScope::AllLanes;
  da_cfg.lateral_inflation      = 0.2;
  da_cfg.longitudinal_inflation = 1.5;

  constexpr double drivable_area_length = 100.0;
  constexpr double drivable_area_before = 10.0;

  const double state_s    = route.get_s( ego, 10.0 ).value_or( 0.0 );
  const double da_start_s = state_s - drivable_area_before;
  const double da_end_s   = state_s + drivable_area_length;

  // Predict all participants (local working copy).
  planner::NominalParticipantPrediction participant_predictor;
  auto                                  participants = domain.traffic_participants;
  participant_predictor.plan_trajectories( participants );

  out.drivable_area = planner::create_drivable_area( route, da_start_s, da_end_s, participants, da_cfg );

  if( !out.drivable_area || out.drivable_area->empty() )
  {
    dynamics::Trajectory fallback;
    fallback.adjust_start_time( ego.time );
    fallback.label          = "Follow Route (No Drivable Area)";
    out.trajectory          = std::move( fallback );
    out.traffic_participant = make_default_participant( domain, planning_tools );
    return out;
  }

  // -----------------------------------------------------------------------------
  // Speed profile
  // -----------------------------------------------------------------------------
  planner::SpeedProfile speed_profile;

  planner::SpeedProfileConfig sp_cfg;
  sp_cfg.total_time        = 5.0;
  sp_cfg.s_horizon         = 100.0;
  sp_cfg.ds_dp             = 0.25;
  sp_cfg.projection_window = 50.0;

  sp_cfg.v_max = 13.6;
  sp_cfg.a_min = -2.0;
  sp_cfg.a_max = 2.0;
  sp_cfg.j_max = 20.0;

  sp_cfg.dt_qp = 0.1;
  sp_cfg.dt_dp = 0.5;

  sp_cfg.w_dp_progress = 1000.0;
  sp_cfg.w_dp_speed    = 1.0;
  sp_cfg.w_dp_accel    = 10.0;
  sp_cfg.w_dp_jerk     = 10.0;
  sp_cfg.w_dp_obstacle = 10.0;

  sp_cfg.max_curvature = 0.1;

  sp_cfg.w_qp_track_dp        = 0.01;
  sp_cfg.w_qp_accel           = 1.0;
  sp_cfg.w_qp_jerk            = 10.0;
  sp_cfg.w_qp_v0              = 10.0;
  sp_cfg.w_qp_a0              = 2.0;
  sp_cfg.w_qp_limit_violation = 100.0;

  speed_profile = planner::plan_speed_profile( *out.drivable_area, participants, ego, sp_cfg );

  std::cerr << "Speed profile points: " << speed_profile.size() << std::endl;

  // -----------------------------------------------------------------------------
  // Reference + initial guess + optimize
  // -----------------------------------------------------------------------------
  const double dt            = 0.1;
  const size_t horizon_steps = 40;

  const dynamics::Trajectory ref_traj = planner::generate_reference_trajectory( speed_profile, out.drivable_area->reference_line, dt,
                                                                                horizon_steps );

  const dynamics::Trajectory guess_traj = planner::initial_guess_pure_pursuit( ref_traj, ego, *planning_tools.vehicle_model );

  // dynamics::Trajectory traj = planning_tools.trajectory_optimizer.optimize_trajectory( ego, ref_traj, guess_traj );
  dynamics::Trajectory traj = ref_traj;


  traj.adjust_start_time( ego.time );
  traj.label = "Follow Route";

  out.trajectory          = std::move( traj );
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
waiting_for_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  // if we have no waypoints, do nothing
  Decision out          = minimum_risk( domain, planning_tools );
  out.trajectory->label = "Waiting for Waypoints";

  if( domain.waypoints.has_value() && domain.waypoints->waypoints.size() > 1 )
  {
    // if we have waypoints, create trajectory to send
    dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, domain.waypoints->waypoints,
                                                                        domain.traffic_participants, *planning_tools.vehicle_model );
    trajectory.label                = "Suggested Trajectory";
    trajectory.adjust_start_time( domain.vehicle_state->time );
    out.trajectory_suggestion = std::move( trajectory );
    out.trajectory->label     = "Waiting for Confirmation";
  }

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
follow_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out;

  dynamics::Trajectory trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, domain.waypoints->waypoints,
                                                                      domain.traffic_participants, *planning_tools.vehicle_model );
  trajectory.label                = "Follow Assistance";
  trajectory.adjust_start_time( domain.vehicle_state->time );
  out.trajectory = std::move( trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  out.assistance_request  = false;
  return out;
}

Decision
safety_corridor( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out; // calculate trajectory getting out of safety corridor
  auto     right_forward_points = planner::filter_points_in_front( domain.safety_corridor->right_border, *domain.vehicle_state );
  auto     safety_waypoints     = planner::shift_points_right( right_forward_points, planning_tools.vehicle_model->params.body_width );
  double   target_speed         = planner::is_point_to_right_of_line( *domain.vehicle_state, right_forward_points ) ? 0 : 2.0;

  auto planned_trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, safety_waypoints, domain.traffic_participants,
                                                              *planning_tools.vehicle_model, target_speed );

  planned_trajectory = planning_tools.trajectory_optimizer.optimize_trajectory( *domain.vehicle_state, planned_trajectory );

  planned_trajectory.label = "Safety Corridor";
  out.trajectory           = std::move( planned_trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
request_assistance( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out            = minimum_risk( domain, planning_tools );
  out.assistance_request  = true;
  out.trajectory->label   = "Request Assistance";
  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

Decision
minimum_risk( const Domain& domain, PlanningParams& planning_tools )
{

  Decision out;

  double state_s            = domain.route->get_s( *domain.vehicle_state, 10.0 ).value_or( 0.0 );
  auto   cut_route          = domain.route->get_shortened_route( state_s, 100.0 );
  auto   planned_trajectory = planner::waypoints_to_trajectory( *domain.vehicle_state, cut_route, domain.traffic_participants,
                                                                *planning_tools.vehicle_model, 0.0 /* target_speed */ );

  planned_trajectory = planning_tools.trajectory_optimizer.optimize_trajectory( *domain.vehicle_state, planned_trajectory );
  if( planned_trajectory.states.size() < 2 )
  {
    out = standstill( domain, planning_tools );
  }
  planned_trajectory.label = "Minimum Risk Maneuver";
  out.trajectory           = std::move( planned_trajectory );

  out.traffic_participant = make_default_participant( domain, planning_tools );
  return out;
}

dynamics::TrafficParticipant
make_default_participant( const Domain& domain, const PlanningParams& planning_tools )
{
  dynamics::TrafficParticipant participant;
  if( domain.vehicle_state )
    participant.state = domain.vehicle_state.value();
  if( domain.route )
  {
    participant.goal_point = domain.route->destination;
    participant.route      = domain.route.value();
  }
  participant.id                  = planning_tools.v2x_id;
  participant.v2x_id              = planning_tools.v2x_id;
  participant.classification      = dynamics::CAR;
  participant.physical_parameters = planning_tools.vehicle_model->params;
  return participant;
}

} // namespace adore::behaviours
