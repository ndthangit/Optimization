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
    # search_parameters.time_limit.FromSeconds(1)
    search_parameters.solution_limit = 100
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found !")


if __name__ == "__main__":
    main()