from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import sys


def create_data_model():
    data = {}

    [data['num_clients'], data['num_vehicles'], data['capacity']] = [int(x) for x in sys.stdin.readline().split()]

    requests = [int(x) for x in sys.stdin.readline().split()]
    requests.insert(0, 0)
    data['requests'] = requests
    distances = []
    for i in range(data['num_clients'] + 1):
        distances.append(list(map(int, sys.stdin.readline().split())))
    data['vehicle_capacities'] = [data['capacity'] for i in range(data['num_vehicles'])]
    data['distance_matrix'] = distances
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(solution.ObjectiveValue())
    # print(f"Objective: {solution.ObjectiveValue()}")
    # total_distance = 0
    # total_load = 0
    # for vehicle_id in range(data["num_vehicles"]):
    #     index = routing.Start(vehicle_id)
    #     plan_output = f"Route for vehicle {vehicle_id}:\n"
    #     route_distance = 0
    #     route_load = 0
    #     while not routing.IsEnd(index):
    #         node_index = manager.IndexToNode(index)
    #         route_load += data["requests"][node_index]
    #         plan_output += f" {node_index} Load({route_load}) -> "
    #         previous_index = index
    #         index = solution.Value(routing.NextVar(index))
    #         route_distance += routing.GetArcCostForVehicle(
    #             previous_index, index, vehicle_id
    #         )
    #     plan_output += f" {manager.IndexToNode(index)} Load({route_load})\n"
    #     plan_output += f"Distance of the route: {route_distance}m\n"
    #     plan_output += f"Load of the route: {route_load}\n"
    #     print(plan_output)
    #     total_distance += route_distance
    #     total_load += route_load
    # print(f"Total distance of all routes: {total_distance}m")
    # print(f"Total load of all routes: {total_load}")

def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()
    # print(data['requests'])
    # print(data['distance_matrix'])
    # print(data['vehicle_capacities'])

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    ## Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return data["requests"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found !")


if __name__ == "__main__":
    main()