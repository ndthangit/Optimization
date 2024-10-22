import sys
from ortools.linear_solver import pywraplp


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

data = create_data_model()
print(data['requests'])

solver = pywraplp.Solver.CreateSolver('SCIP')

x = [[solver.IntVar(0, 1, 'x(' + str(i) + ',' + str(j) + ')') for j in range(num_teacher)] for i in range(num_course)]
