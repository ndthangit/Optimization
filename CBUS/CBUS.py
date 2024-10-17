from ortools.constraint_solver import pywrapcp, routing_enums_pb2

import sys
def create_data_model():
    data = {}
    # Read number of passengers and vehicle capacity
    [data["num_passenger"], data["vehicle_capacity"]] = [int(x) for x in sys.stdin.readline().split()]
    # Initialize distance matrix
    distance = []
    for i in range(2 * data["num_passenger"] + 1):
        distance.append([int(x) for x in sys.stdin.readline().split()])

    # Prepare deliveries list (pairs of pickup and delivery points)
    deliveries = []
    for i in range(1, data["num_passenger"] + 1):
        deliveries.append([i, i + data["num_passenger"]])

    # Store number of vehicles, deliveries, distance matrix, and depot in data
    data["num_vehicles"] = 1  # Assuming 1 vehicle is used for now
    data["deliveries"] = deliveries
    data["distance"] = distance
    data["depot"] = 0
    data["curCapacity"] = 0

    return data


def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()} miles")
    index = routing.Start(0)
    plan_output = "Route for vehicle 0:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += f" {manager.IndexToNode(index)}\n"
    print(plan_output)
    plan_output += f"Route distance: {route_distance}miles\n"


data = create_data_model()
# print(data["distance"])

manager = pywrapcp.RoutingIndexManager(
    len(data["distance"]), data["num_vehicles"], data["depot"]
)
routing = pywrapcp.RoutingModel(manager)

# demand_callback_index = routing.RegisterUnaryTransitCallback(data["curCapacity"])

# routing.AddDimensionWithVehicleCapacity(
#     demand_callback_index,
#     0,  # null capacity slack
#     data["vehicle_capacity"],  # vehicle maximum capacities
#     True,  # start cumul to zero
#     "Capacity",
# )

def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data["distance"][from_node][to_node]


transit_callback_index = routing.RegisterTransitCallback(distance_callback)

# Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)



# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)

print_solution(manager, routing, solution)
# print(data["distance"][0][3])
