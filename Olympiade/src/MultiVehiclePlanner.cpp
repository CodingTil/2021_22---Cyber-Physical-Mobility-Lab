#include "MultiVehiclePlanner.h"

MultiVehiclePlanner::MultiVehiclePlanner(VisualizationHandlerPtr visualization, LaneGraphPtr lane_graph)
	: UserPlanner(std::move(visualization), std::move(lane_graph)) {}

void MultiVehiclePlanner::UpdatePlanner(Time step_size, Time /*planning_time*/) {
	auto update_function = [this, step_size](const auto &planner_pair) {
		Id id = planner_pair.first;
		VehiclePlannerPtr planner = planner_pair.second;

		// Is planner valid if not skip
		if (!planner)
			return;


		VehicleState cur = planner->GetCurrentVehicleState();
		// std::cout << static_cast <int> (id) << std::endl;
		bool change = false;
		Pos pos = cur.GetPosition();
		// std::cout << pos << std::endl;
		Vel velo = cur.GetVelocity();
		for (auto obstacle : this->GetObstacles()) {
			// Avoid Collisions with objects.
			Pos obstacle_position = obstacle.second.GetPosition();
			double distance = std::sqrt(std::pow(obstacle_position.x() - pos.x(), 2) + std::pow(obstacle_position.y() - pos.y(), 2));
			if (distance < 0.7) {
				change = true;
			}
		}


		if (change == false) {
			planner->Update(step_size);
			this->AddVehicleState(id, planner->GetCurrentVehicleState(), "MultiVehiclePlanner");
		} else {
			// Collision
			planner->UpdateTimer(step_size);
			cur.SetTime(cur.GetTime() + step_size);
			cur.SetVelocity({0.0, 0.0});
			this->AddVehicleState(id, cur, "MultiVehiclePlanner");

			//std::cout << cur.GetVelocity() << std::endl;
			//std::cout << planner->GetCurrentVehicleState().GetVelocity() << std::endl;
			//std::cout << "Position" << cur.GetPosition() << std::endl;
			//auto planned_states = planner->GetPlannedVehicleStates();
			// planned_states.erase(planned_states.begin(),planned_states.end());
			// planned_states->clear();
			//std::cout << planned_states->size() << std::endl;
			//std::cout << planner->GetPlannedVehicleStates()->size() << std::endl;
			// planner->RecalculateRoute();


			/*Vel vel = {0.0, 0.0};
			planner->UpdateCurrentState(pos, cur.GetLaneletId(), vel);
			this->AddVehicleState(id, planner->GetCurrentVehicleState(), "MultiVehiclePlanner");
			*/
		}




		// this->AddVehicleState(id, planner->GetCurrentVehicleState(), "MultiVehiclePlanner");


		// If planner finished stop planning
		if (planner->IsTargetPositionReached()) {
			this->vehicle_planner_[id] = nullptr;
			VehicleReachedTarget(id);
			//      std::cout << "[MULTIVEHICLEPLANNER] Vehicle " << int(id) << " marked for pickup by system at: " << planning_time << std::endl;
		}
	};
	std::for_each(this->vehicle_planner_.begin(), this->vehicle_planner_.end(), update_function);
}

void MultiVehiclePlanner::VehicleAvailable(const Id id, const VehicleState state, const Pos target_pos) {
	std::cout << "[MULTIVEHICLEPLANNER] Detected vehicle." << std::endl;
	auto planner = new VehiclePlanner(id, this->GetLaneGraph(), this->GetVisualization());
	planner->InitialiseVehicleState(state.GetLaneletId(), state.GetPosition(), state.GetVelocity(), state.GetTime());
	planner->CalculateRouteToTargetPosition(target_pos);
	this->vehicle_planner_[id] = std::shared_ptr<VehiclePlanner>(planner);
}

void MultiVehiclePlanner::ObstacleDetected(Id id, ObstacleState state) {
	std::cout << "[MULTIVEHICLEPLANNER] Detected obstacle." << std::endl;
}