#include "DifferentialDriveGuidance.hpp"

DifferentialDriveGuidance::DifferentialDriveGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();

	pid_init(&_heading_pid, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_init(&_velocity_pid, PID_MODE_DERIVATIV_NONE, 0.001f);
}

matrix::Vector2f DifferentialDriveGuidance::computeGuidance(const matrix::Vector2d &global_pos,
		const matrix::Vector2d &current_waypoint, const matrix::Vector2d &next_waypoint,
		float vehicle_yaw, float body_velocity, float angular_velocity, float dt)
{
	const float distance_to_next_wp = get_distance_to_next_waypoint(global_pos(0), global_pos(1), current_waypoint(0),
					  current_waypoint(1));

	float desired_heading = get_bearing_to_next_waypoint(global_pos(0), global_pos(1), current_waypoint(0),
				current_waypoint(1));
	float heading_error = matrix::wrap_pi(desired_heading - vehicle_yaw);

	const float max_velocity = math::trajectory::computeMaxSpeedFromDistance(_param_rdd_max_jerk.get(),
				   _param_rdd_max_accel.get(), distance_to_next_wp, 0.0f);

	_forwards_velocity_smoothing.updateDurations(max_velocity);
	_forwards_velocity_smoothing.updateTraj(dt);

	// Make rover stop when it arrives at the last waypoint instead of loitering and driving around weirdly.
	if ((current_waypoint == next_waypoint) && distance_to_next_wp < _param_nav_acc_rad.get()) {
		currentState = GuidanceState::GOAL_REACHED;

	} else if (_next_waypoint != next_waypoint) {
		if (fabsf(heading_error) < 0.1f) {
			currentState = GuidanceState::DRIVING;

		} else {
			currentState = GuidanceState::TURNING;
		}

	} else {
		currentState = GuidanceState::DRIVING;
	}

	matrix::Vector2f output;
	float desired_speed = 0.f;

	switch (currentState) {
	case GuidanceState::TURNING:
		desired_speed = 0.f;
		break;

	case GuidanceState::DRIVING:
		desired_speed = math::interpolate<float>(abs(heading_error), 0.1f, 0.2f,
				_forwards_velocity_smoothing.getCurrentVelocity(), 0.0f);
		break;

	case GuidanceState::GOAL_REACHED:
		// temporary till I find a better way to stop the vehicle
		desired_speed = 0.f;
		body_velocity = 0.f;
		heading_error = 0.f;
		angular_velocity = 0.f;
		_desired_angular_velocity = 0.f;
		break;
	}

	float speed_pid = pid_calculate(&_velocity_pid, desired_speed, body_velocity, 0, dt);
	float angular_velocity_pid = pid_calculate(&_heading_pid, heading_error, angular_velocity, 0, dt);

	desired_speed += speed_pid;
	_desired_angular_velocity += angular_velocity_pid;

	output(0) = desired_speed;
	output(1) = _desired_angular_velocity;

	return output / _max_speed;
}

void DifferentialDriveGuidance::updateParams()
{
	ModuleParams::updateParams();

	pid_set_parameters(&_heading_pid,
			   _param_rdd_p_gain_heading.get(),  // Proportional gain
			   _param_rdd_i_gain_heading.get(),  // Integral gain
			   0,  // Derivative gain
			   20,  // Integral limit
			   200);  // Output limit

	pid_set_parameters(&_velocity_pid,
			   _param_rdd_p_gain_speed.get(),  // Proportional gain
			   _param_rdd_i_gain_speed.get(),  // Integral gain
			   0,  // Derivative gain
			   2,  // Integral limit
			   200);  // Output limit

	_forwards_velocity_smoothing.setMaxJerk(_param_rdd_max_jerk.get());
	_forwards_velocity_smoothing.setMaxAccel(_param_rdd_max_accel.get());
	_forwards_velocity_smoothing.setMaxVel(_max_speed);
}
