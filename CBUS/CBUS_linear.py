import sys
from ortools.linear_solver import pywraplp


def create_data_model():
    data = {}
    # Read number of passengers and vehicle capacity
    [n, k] = [int(x) for x in sys.stdin.readline().split()]
    # Initialize distance matrix
    data['num_passenger'] = n
    data['capacity'] = k
    distance = []
    for i in range(2 * data['num_passenger'] + 1):
        distance.append([int(x) for x in sys.stdin.readline().split()])

    # Prepare deliveries list (pairs of pickup and delivery points)
    deliveries = []
    for i in range(1, data['num_passenger'] + 1):
        deliveries.append([i, i + data['num_passenger']])
    # Store number of vehicles, deliveries, distance matrix, and depot in data
    data['num_vehicles'] = 1  # Assuming 1 vehicle is used for now
    data['deliveries'] = deliveries
    data['distance'] = distance
    data['depot'] = 0
    data['curCapacity'] = 0

    return data


data = create_data_model()
print(data['distance'])
print(data['num_passenger'])
solver = pywraplp.Solver.CreateSolver('SCIP')

infinity = solver.infinity()

# route
x = [[solver.IntVar(0, 1, 'x(' + str(i) + ',' + str(j) + ')') for j in range(2 * data['num_passenger'] + 1)] for i in
     range(2 * data['num_passenger'] + 1)]

# make sure have only one route
y = [solver.IntVar(0, 2* data['capacity'], 'y(' + str(i) + ')') for i in range(2 * data['num_passenger'] + 1)]

# num passengers at each point
u = [solver.IntVar(0,data['capacity'] , 'u(' + str(i) + ')') for i in range(2 * data['num_passenger'] + 1)]

z = solver.IntVar(0, infinity, 'z')

for i in range(2 * data['num_passenger'] + 1):
    for j in range(2 * data['num_passenger'] + 1):
        '''
        each route can be done or not
        '''
        if i!=j:
            c = solver.Constraint(0, 1)
            c.SetCoefficient(x[i][j], 1)
            c.SetCoefficient(x[j][i], 1)
        else:
            c = solver.Constraint(0, 0)
            c.SetCoefficient(x[i][j], 1)
            # c.SetCoefficient(x[j][i], 1)

'''
1 point can only be visited once
'''

for i in range(2 * data['num_passenger'] + 1):
    c = solver.Constraint(1, 1)
    for j in range(2 * data['num_passenger'] + 1):
        c.SetCoefficient(x[i][j], 1)

for i in range(2 * data['num_passenger'] + 1):
    c = solver.Constraint(1, 1)
    for j in range(2 * data['num_passenger'] + 1):
        c.SetCoefficient(x[j][i], 1)

'''Bus Capacity Constraint'''

for i in range(1, data['num_passenger'] + 1):
    c = solver.Constraint(1, infinity)
    c.SetCoefficient(y[i], -1)
    c.SetCoefficient(y[i + data['num_passenger']], 1)


'''Subtour Elimination Constraint'''
for i in range(1, 2 * data['num_passenger'] + 1):
    for j in range(1, 2 * data['num_passenger'] + 1):
        if i != j:
            c = solver.Constraint(-100, 2 * data['num_passenger'])
            c.SetCoefficient(y[i], 1)
            c.SetCoefficient(y[j], -1)
            c.SetCoefficient(x[i][j], 2 * data['num_passenger'] + 1)

#
# for i in range(1, data['num_passenger'] + 1):
#     for j in range(data['num_passenger']+1, 2*data['num_passenger'] + 1):
#         if(x[i][j] > 0):
#             c = solver.Constraint(1, 1)
#             c.SetCoefficient(u[i], 1)
#             c.SetCoefficient(u[j], -1)
#
# for i in range(1, data['num_passenger'] + 1):
#     for j in range(data['num_passenger']+1, 2*data['num_passenger'] + 1):
#         if(x[i][j] > 0):
#             c = solver.Constraint(-1, -1)
#             c.SetCoefficient(u[i], 1)
#             c.SetCoefficient(u[j], -1)



'''purpose function'''
purpose = solver.Constraint(0, infinity)
for i in range(2 * data['num_passenger'] + 1):
    for j in range(2 * data['num_passenger'] + 1):
        purpose.SetCoefficient(x[i][j], -data['distance'][i][j])
purpose.SetCoefficient(z, 1)

obj = solver.Objective()
obj.SetCoefficient(z, 1)
obj.SetMinimization()
status = solver.Solve()
obj = solver.Objective()
for i in range(2 * data['num_passenger'] + 1):
    for j in range(2 * data['num_passenger'] + 1):
        obj.SetCoefficient(x[i][j], data['distance'][i][j])
obj.SetMinimization()
status = solver.Solve()

if status == pywraplp.Solver.OPTIMAL:
    # print('Objective value =', solver.Objective().Value())
    print('Number of variables =', solver.NumVariables())
    for i in range(2 * data['num_passenger'] + 1):
        for j in range(2 * data['num_passenger'] + 1):
            if x[i][j].solution_value() > 0:
                print('x[', i, ',', j, '] =', x[i][j].solution_value())
    print(int(z.solution_value()))
