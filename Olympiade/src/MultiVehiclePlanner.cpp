#include "MultiVehiclePlanner.h"
#include <string>

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
		Vel vel = cur.GetVelocity();

		// Obstacle Collision
		for (auto obstacle : this->GetObstacles()) {
			// Avoid Collisions with objects.
			Pos obstacle_position = obstacle.second.GetPosition();
			double distance = std::sqrt(std::pow(obstacle_position.x() - pos.x(), 2) + std::pow(obstacle_position.y() - pos.y(), 2));
			if (distance < 0.7) {
				std::cout << "Obstacle Collision with Vehicle " << id << std::endl;
				change = true;
			}
		}

		// Vehicle Collision - Main Approach
		/*for (auto vehicle_2 : this->vehicle_planner_) {
			Id id2 = vehicle_2.first;
			VehiclePlannerPtr planner2 = vehicle_2.second;
			Pos pos2 = planner2->GetCurrentVehicleState().GetPosition();
			double distance = std::sqrt(std::pow(pos2.x() - pos.x(), 2) + std::pow(pos2.y() - pos.y(), 2));
			if (distance < 0.2) {
				auto planned_states_1 = planner->GetPlannedVehicleStates();
				auto planned_states_2 = planner2->GetPlannedVehicleStates();
				bool breakloop = false;
				for (int index_2 = 0; index_2 < planned_states_2.size(); index_2++) {
					if (breakloop)
						break;
					auto lanelet_id_2 = planned_states_2[index_2].GetLaneletId();
					for (int index_1 = 0; index_1 < planned_states_1.size(); index_1++) {
						auto lanelet_id_1 = planned_states_1[index_1].GetLaneletId();
						if (lanelet_id_1 == lanelet_id_2) {
							if (index_1 > index_2) {
								std::cout << "Vehicle " << id << " collided with Vehicle " << id2 << std::endl;
								change = true;
							}
							breakloop = true;
							break;
						}
					}
				}
			}
		}*/

		/* IDEA - ALSO NOT WORKING
		auto speed = cur.GetSpeed();
		for (auto vehicle_2 : this->vehicle_planner_) {
			if (change)
				break;
			Id id2 = vehicle_2.first;
			if(id == id2) {
				continue;
			}
			VehiclePlannerPtr planner2 = vehicle_2.second;
			Pos pos2 = planner2->GetCurrentVehicleState().GetPosition();
			auto speed2 = planner2->GetCurrentVehicleState().GetSpeed();
			auto planned_states_1 = planner->GetPlannedVehicleStates();
			auto planned_states_2 = planner2->GetPlannedVehicleStates();

			auto last_lanelet_id = planner2->GetCurrentVehicleState().GetLaneletId();
			for (auto index = 0; index < planned_states_2.size(); index++) {
				if (change)
					break;

				auto lanelet_id_2 = planned_states_2[index].GetLaneletId();
				if(last_lanelet_id == lanelet_id_2) {
					continue;
				}else {
					last_lanelet_id = lanelet_id_2;
				}

				for (auto offset = 1; offset <= 3 && index + offset < planned_states_1.size(); offset++) {
					auto lanelet_id_1_offset = planned_states_1[index + offset].GetLaneletId();
					if (lanelet_id_1 == lanelet_id_2_offset) {
						if(speed != 0 && speed2 != 0) {
							change = true; // we are car behind
						}else if(speed == 0 && speed2 != 0) {
							change = true; // still wait
						}else if(speed != 0 && speed2 == 0) {
							change = true; // car in front of us standing
						}
					}

					if(change) {
						std::cout << "Vehicle Collision between " << std::to_string(id) << " and " << std::to_string(id2) << std::endl;
					}
				}
			}
		}
		*/

		// Collision Handling
		if (change == false) {
			planner->Update(step_size);
			this->AddVehicleState(id, planner->GetCurrentVehicleState(), "MultiVehiclePlanner");
		} else {
			// Collision
			planner->UpdateTimer(step_size);
			cur.SetTime(cur.GetTime() + step_size);
			cur.SetVelocity({0.0, 0.0});
			this->AddVehicleState(id, cur, "MultiVehiclePlanner");
			Pos pos = cur.GetPosition();
			LaneletId lanelet_id = cur.GetLaneletId();
			Vel vel = cur.GetVelocity();
			planner->UpdateCurrentState(pos, lanelet_id, vel);
		}

		// If planner finished stop planning
		if (planner->IsTargetPositionReached()) {
			this->vehicle_planner_[id] = nullptr;
			VehicleReachedTarget(id);
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