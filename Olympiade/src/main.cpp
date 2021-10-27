#include <cpm_routing/projects/ScenarioProject.h>
#include "MultiVehiclePlanner.h"

int main(int argc, char *argv[]) {
	std::cout << "[MAIN] Defining parameter" << std::endl;
	const std::string kName = "R001";

	std::cout << "[MAIN] Loading project" << std::endl;
	cpm_routing::ScenarioProjectPtr project = cpm_routing::LoadScenario(kName, argc, argv);

	std::cout << "[MAIN] Creating planner" << std::endl;
	MultiVehiclePlannerPtr planner = std::make_shared<MultiVehiclePlanner>(project->GetVisualization(), project->GetLaneGraph());
	project->CreatePlanner(planner);

	std::cout << "[MAIN] Executing project" << std::endl;
	try {
		project->Execute();
	} catch (const std::exception &e) {
		std::cerr << "[ERROR] Project crashed with message: " << e.what() << std::endl;
		return 1;
	}

	project->Evaluate();

	project->Terminate();

	std::cout << "[MAIN] Terminating application" << std::endl;
	return 0;
}