import sys
from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model


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

data = create_data_model()
n = data['num_passenger']

model = cp_model.CpModel()

# route
x = [[model.NewIntVar(0, 1, 'x(' + str(i) + ',' + str(j) + ')') for j in range(2 * n + 1)] for i in
     range(2 * n + 1)]

# make sure have only one route
y = [model.NewIntVar(0, 2 * n, 'y(' + str(i) + ')') for i in range(2 * n + 1)]

# num passengers at each point
u = [model.NewIntVar(0, data['capacity'], 'u(' + str(i) + ')') for i in range(2 * n + 1)]

for i in range(1, 2 * n + 1):
    model.Add(sum(x[i][j] for j in range(1,2 * n + 1) if i != j) == 1)
    model.Add(sum(x[j][i] for j in range(2 * n + 1) if i != j) == 1)

model.Add(u[0] == 0)
model.Add(y[0] == 0)

# Ensure deliveries happen after pickups
for i in range(1, n + 1):
    model.Add(y[i] < y[i + n])

# Capacity constraints
for i in range(2 * n + 1):
    model.Add(u[i] <= data['capacity'])

# Subtour elimination constraints to ensure a valid route
for i in range(2 * n + 1):
    for j in range(1, 2 * n + 1):
        if i != j:
            model.Add(y[i] - y[j] + (2 * n + 1) * x[i][j] <= 2 * n)

for i in range(2 * n + 1):
    for j in range(2 * n + 1):
        if i != j:
            model.Add(x[i][j] + x[j][i] <= 1)
        else:
            model.Add(x[i][j] == 0)

for i in range(1, 2 * n + 1):
    for j in range(1, n + 1):
        if i != j:
            model.Add((u[i] + 1 - u[j]) == 0).OnlyEnforceIf(x[i][j])
    for j in range(n + 1, 2 * n + 1):
        if i != j:
            model.Add((u[i] - 1 - u[j]) == 0).OnlyEnforceIf(x[i][j])

purpose = sum(data['distance'][i][j] * x[i][j] for i in range(2 * n + 1) for j in range(2 * n + 1) if i != j)
model.Minimize(purpose)
solver = cp_model.CpSolver()
solution = solver.Solve(model)

if solution == cp_model.OPTIMAL:
    print(int(solver.ObjectiveValue()))
else:
    print("No solution found")
