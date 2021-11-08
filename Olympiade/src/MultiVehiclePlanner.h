/**
 * @author Simon Schaefer
 * @date 31.05.2021
 */

#ifndef MULTIVEHICLEPLANNER_H_
#define MULTIVEHICLEPLANNER_H_

#include <cpm_routing/cpm_routing.h>
#include "VehiclePlanner.h"

using namespace cpm_routing;

/**
 * User Implementation of the planner that will be triggered by the routing lib.
 */
class MultiVehiclePlanner : public UserPlanner {
   private:
	VehiclePlannerMap vehicle_planner_; ///< List of planes associated to the physical vehicle.

   public:
	/**
	 * Passthrough to the planner constructor.
	 * @param update_rate Rate with which this planner will be updated.
	 */
	explicit MultiVehiclePlanner(VisualizationHandlerPtr visualization, LaneGraphPtr lane_graph);

	/**
	 * Main loop function that will be triggered according to the update rate.
	 */
	void UpdatePlanner(Time step_size, Time planning_time) override;

	/**
	 * Interrupt function that will be triggered if a new vehicle is available for pickup. Make sure to pick it up before
	 * the pickup time is reached.
	 * @param state Pickup vehicle state
	 * @param target_pos Target position of the vehicle.
	 */
	void VehicleAvailable(Id id, VehicleState state, Pos target_pos) override;

	void ObstacleDetected(Id id, ObstacleState state) override;
};
typedef std::shared_ptr<MultiVehiclePlanner> MultiVehiclePlannerPtr;

#endif // MULTIVEHICLEPLANNER_H_
