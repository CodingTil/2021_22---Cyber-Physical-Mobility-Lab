#include "VehiclePlanner.h"

using namespace cpm_routing;

void VehiclePlanner::TravelAlongPlannedStates(Time delta_t) {
	// Time the vehicle traveled since the last update
	Time remaining_time = delta_t;

	// Push current state on planned state buffer but ensure deletion at end of
	// this functions
	long reached_states = 1;
	this->planned_states_.insert(this->planned_states_.begin(),
								 this->current_state_);

	// Check how many states where covered since last update
	for (size_t i = 0; i < this->planned_states_.size() - 1; i++) {
		auto state = this->planned_states_.at(i);
		auto next_state = this->planned_states_.at(i + 1);

		// Get time needed to cover state transition
		double speed = InterpolateSpeed(state, next_state, 0.5);
		double distance = Distance(state, next_state);
		auto time_in_state =
			speed != 0 ? static_cast<Time>(distance / speed * 1e9) : 0ull;

		// Percent of distance covered between two states
		double factor = std::min(static_cast<double>(remaining_time) /
									 static_cast<double>(time_in_state),
								 1.0);
		if (factor >= 1.0) {
			// 100% distance between states covered update buffer and iterate to next
			// state
			remaining_time -=
				static_cast<Time>(static_cast<double>(time_in_state) * factor);
			reached_states++;
			continue;
		}

		// Not 100% distance between states covered update current position and
		// break loop
		auto position = InterpolatePosition(state, next_state, factor);
		auto velocity = InterpolateVelocity(state, next_state, factor);
		auto lanelet_id = next_state.GetLaneletId();
		this->UpdateCurrentState(position, lanelet_id, velocity);
		break;
	}

	// Erase old states
	this->planned_states_.erase(this->planned_states_.begin(),
								this->planned_states_.begin() + reached_states);
}

void VehiclePlanner::AddState(VehicleState state) {
	// Calculate speed value if multiple states are present
	if (!this->planned_states_.empty()) {
		VehicleState &last_state = this->planned_states_.back();
		Vel velocity_vector = Accelerate(last_state, state, 1.0, 1.0);
		state.SetVelocity(velocity_vector);
		last_state.SetVelocity(velocity_vector);
	}
	// Push state to planned states
	planned_states_.push_back(state);
}

void VehiclePlanner::UpdateCurrentState(Pos &position, long lanelet_id, const Vel &vel) {
	this->current_state_.SetPosition(position);
	this->current_state_.SetLaneletId(lanelet_id);
	this->current_state_.SetTime(this->time_);
	this->current_state_.SetVelocity(vel);
	this->vis_->ShowPoint(this->vehicle_id_ * 1000 + 2, position, 0.05,
						  VisualizationHandler::RED);
}

VehiclePlanner::VehiclePlanner(const Id vehicle_id, LaneGraphPtr map, VisualizationHandlerPtr vis)
	: vehicle_id_(vehicle_id), vis_(std::move(vis)), map_(std::move(map)), target_position_reached_(false) {}

void VehiclePlanner::InitialiseVehicleState(LaneletId lanelet_id,
											const Pos &pos,
											const Vel &vel,
											Time time) {
	// Remove all old states
	this->planned_states_.clear();

	// Initialise current state
	this->time_ = time;
	this->current_state_ = {time, lanelet_id, pos, vel, "VehiclePlanner"};
	this->AddState(current_state_);
}

void VehiclePlanner::UpdateTimer(Time delta_t) {
	// Update time
	this->time_ += delta_t;
}

void VehiclePlanner::Update(Time delta_t) {
	// Update time
	UpdateTimer(delta_t);

	// Do not update if target is reached
	if (this->target_position_reached_)
		return;

	// Remove visited states and update current position
	this->TravelAlongPlannedStates(delta_t);

	// Extend route if planned trajectory reaches end
	if (this->planned_states_.empty()) {
		// std::cout << "[VEHICLEPLANNER] No Planned States for Vehicle " << this->vehicle_id_ << "." << std::endl;
		if (this->current_state_.GetPosition() == this->target_pos_ || true) {
			this->target_position_reached_ = true;
			std::cout << "[VEHICLEPLANNER] Vehicle " << this->vehicle_id_ << " has reached its target." << std::endl;
		} else {
			this->RecalculateRoute();
		}
		return;
	}
}

VehicleState VehiclePlanner::GetCurrentVehicleState() const {
	return this->current_state_;
}

VehicleStates VehiclePlanner::GetPlannedVehicleStates() {
	return planned_states_;
}

void VehiclePlanner::CalculateRouteToTargetPosition(const Pos &target_position) {
	this->target_pos_ = target_position;
	this->target_position_reached_ = false;
	this->RecalculateRoute();
}

void VehiclePlanner::RecalculateRoute() {
	if (this->target_position_reached_)
		return;
	if (this->planned_states_.empty())
		this->AddState(current_state_);

	this->vis_->ShowPoint(vehicle_id_ * 1000 + 1, this->target_pos_, 0.05,
						  VisualizationHandler::GREEN);

	// Get route to target from lanelet map
	auto last_planned_lanelet =
		this->map_->FindLaneletById(this->planned_states_.back().GetLaneletId());
	auto target_lanelet = this->map_->FindLaneletByPosition(this->target_pos_);
	auto route_to_target =
		this->map_->FindShortestRoute(last_planned_lanelet, target_lanelet);
	auto shortest_path =
		route_to_target->remainingShortestPath(last_planned_lanelet);

	auto add_center_line_to_planned_states = [this](const auto &next_lanelet) {
		// Do not add the same lanelet twice
		if (next_lanelet.id() == this->planned_states_.back().GetLaneletId())
			return;
		auto center_line = next_lanelet.centerline();
		for (size_t j = 0; j < center_line.size() - 1; j++) {
			VehicleState state = {this->time_,
								  next_lanelet.id(),
								  center_line[j].basicPoint2d(),
								  {0.0, 0.0},
								  "VehiclePlanner"};
			this->AddState(state);
		}
	};

	std::for_each(shortest_path.begin(), shortest_path.end(),
				  add_center_line_to_planned_states);

	//this->planned_states_.erase(this->planned_states_.end() - 1, this->planned_states_.end());

	VehicleState state = {this->time_,
						  target_lanelet.id(),
						  this->target_pos_,
						  {0.0, 0.0},
						  "VehiclePlanner"};
	this->AddState(state);
}

bool VehiclePlanner::IsTargetPositionReached() const {
	return this->target_position_reached_;
}