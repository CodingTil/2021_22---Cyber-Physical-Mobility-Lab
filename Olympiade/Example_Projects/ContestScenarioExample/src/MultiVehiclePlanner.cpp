#include "MultiVehiclePlanner.h"

MultiVehiclePlanner::MultiVehiclePlanner(VisualizationHandlerPtr visualization, LaneGraphPtr lane_graph)
: UserPlanner(std::move(visualization), std::move(lane_graph)) {}

void MultiVehiclePlanner::UpdatePlanner(Time step_size, Time /*planning_time*/) {
  auto update_function = [this, step_size](const auto &planner_pair) {
    Id id = planner_pair.first;
    VehiclePlannerPtr planner = planner_pair.second;

    // Is planner valid if not skip
    if (!planner) return;

    // Update planner
    planner->Update(step_size);
    this->AddVehicleState(id, planner->GetCurrentVehicleState(), "MultiVehiclePlanner");

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
