/**
 * @author Simon Schaefer
 * @date 31.05.2021
 */

#ifndef VEHICLEPLANNER_H_
#define VEHICLEPLANNER_H_

#include <cpm_routing/cpm_routing.h>

using namespace cpm_routing;

/**
 * This planner will navigate a single vehicle through the lane graph. This plan is designed to be dynamically
 * allocated an deleted of target point is reached.
 */
class VehiclePlanner {
   private:
	Pos target_pos_;
	bool target_position_reached_; ///< Indicates that the target is reached and no updates are needed.

	const VisualizationHandlerPtr vis_; ///< Visualization to show information in supported visualizations.
	const LaneGraphPtr map_;			///< LaneGraph to navigate the vehicle on.

	const Id vehicle_id_; ///< Id of the physical vehicle.

	Time time_ = 0;				   ///< Time that is increased by each step.
	VehicleStates planned_states_; ///< Planned future state.
	VehicleState current_state_;   ///< Current state.


	/**
	 * Calculated the distance that was driven in a given time.
	 * @param delta_t Traveled time since last update.
	 */
	void TravelAlongPlannedStates(Time delta_t);

	/**
	 * Adds state to the planned stated list. This function will also calculate the optimal speed for the state based on
	 * the previous states.
	 * @param state State to add to the planned states. Velocity will be overwritten.
	 */
	void AddState(VehicleState state);

   public:
	/**
	 * Initialises a new planner and initialises the pointer to all helper classes.
	 * @param vehicle_id Identifier of vehicle
	 * @param map Lane graph to travel on.
	 * @param vis Visualisation to use is supported.
	 */
	VehiclePlanner(Id vehicle_id, LaneGraphPtr map, VisualizationHandlerPtr vis);

	/**
	 * Initialises the vehicle state. This function must be called before the first update.
	 * @param lanelet_id Lanelet the vehicle was matched to.
	 * @param pos Position of the vehicle.
	 * @param vel Velocity of the vehicle.
	 * @param time Time of the detection.
	 */
	void InitialiseVehicleState(LaneletId lanelet_id, const Pos &pos, const Vel &vel, Time time);

	/**
	 * Updates the current plan until the target is reached than calculates a new route to next target.
	 * @param delta_t Time elapsed since last update.
	 */
	void Update(Time delta_t);

	/**
	 * Updates the current position and timing to match the position and lanelet given.
	 * @param position New position.
	 * @param lanelet_id New lanelet.
	 */
	void UpdateCurrentState(Pos &position, long lanelet_id, const Vel &vel);

	/**
	 * Getter for the current state. Will be changed by an update.
	 * @return Current state of the vehicle according to this plan.
	 */
	[[nodiscard]] VehicleState GetCurrentVehicleState() const;

	/**
	 * This is insecure as fuck by I don't really care no more.
	 * Getter for the planned states. Will be changed by an update.
	 * @return Planned states of the vehicle.
	 */
	VehicleStates* GetPlannedVehicleStates();

	/**
	 * Calculates a route to the given target.
	 * @param target_position Target to reach.
	 */
	void CalculateRouteToTargetPosition(const cpm_routing::Pos &target_position);

	/**
	 * Recalculates the route to the stored target.
	 */
	void RecalculateRoute();

	/**
	 * Getter for the target_position_reached_ flag.
	 * @return True if target point is reached, false otherwise.
	 */
	[[nodiscard]] bool IsTargetPositionReached() const;
};
typedef std::shared_ptr<VehiclePlanner> VehiclePlannerPtr; ///< Smart pointer to allocate planner.
typedef std::map<Id, VehiclePlannerPtr> VehiclePlannerMap; ///< Map to match planner to vehicle id.

#endif // VEHICLEPLANNER_H_
