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

    VehicleState currentState = planner->GetCurrentVehicleState();
    double dist = 0;
    // Try to detect collisions with obstacles
    bool collisiondetected = false;
    Pos curStatePos = currentState.GetPosition();
    //Vel velo = currentState.GetVelocity();
    for (auto obstacle : this->GetObstacles()) {
      // Avoid Collisions with objects.
      Pos obstacle_position = obstacle.second.GetPosition();
      double distance = std::sqrt(std::pow(obstacle_position.x() - curStatePos.x(), 2) + std::pow(obstacle_position.y() - curStatePos.y(), 2));
      dist = distance;
      if (distance < 0.6) {
        collisiondetected = true;
      }else{
        collisiondetected = false;
      }
    }

    // handle the action of the car (vehicleStates)
    if (collisiondetected ) {
      // handle collision with obstacle (stop the vehicle until the obstacle moved away)
      std::cout << "Collision " << int(id) << std::endl;
      planner->GetCurrentVehicleState().SetTime(planner->GetCurrentVehicleState().GetTime() + step_size);
      planner->GetCurrentVehicleState().SetVelocity({0.0, 0.0});
      std::cout << "Kollision erkannt" << planner->GetCurrentVehicleState().GetPosition() << std::endl;
      planner->Update(0);
      planner->UpdateTimer(step_size);
    }
    else {
      // everything fine! Just keep going.
      std::cout << "Update Planner " << int(id) << std::endl;
      std::cout << "------------------------------------------------"<< std::endl;
      std::cout << "Vor dem Update" << planner->GetCurrentVehicleState().GetPosition() << std::endl;
      planner->Update(step_size);
      std::cout << "Nach dem Update" << planner->GetCurrentVehicleState().GetPosition() << std::endl;
      std::cout << "------------------------------------------------"<< std::endl;
      
    }
    std::cout << "Am Ende" << planner->GetCurrentVehicleState().GetPosition() << std::endl;
    this->AddVehicleState(id, planner->GetCurrentVehicleState(), "MultiVehiclePlanner");

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