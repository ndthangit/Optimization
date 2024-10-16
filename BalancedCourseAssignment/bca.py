from ortools.linear_solver import pywraplp
import sys


def input():
    [m, n] = [int(x) for x in sys.stdin.readline().split()]
    A = [] # preference matrix
    for i in range(m):
        A.append([int(x)-1 for x in sys.stdin.readline().split()[1:]])
    [K] = [int(x) for x in sys.stdin.readline().split()]

    B = [] # conflict pairs
    for k in range(K):
        [i, j] = [int(x)-1 for x in sys.stdin.readline().split()]
        B.append([i, j])
    return m, n, A, B


num_teacher, num_course, teacher, conflict = input()
# print(teacher) #  teacher teach teacher[1:]
# print (conflict)

solver = pywraplp.Solver.CreateSolver('SCIP')

# the course is taught by teacher, isn't it?
x = [[solver.IntVar(0, 1, 'x(' + str(i) + ',' + str(j) + ')') for j in range(num_teacher)] for i in range(num_course)]
z = solver.IntVar(0, num_course, 'z')

for t in range(num_teacher):
    for i in range(num_course):
        '''
        if course i is not in teacher t's preference list,
        '''
        if not (i in teacher[t]):
            c = solver.Constraint(0, 0)
            c.SetCoefficient(x[i][t], 1)

for [i, j] in conflict:
    '''
    if course i and course j are conflict, 
    then they can't be taught by the same teacher
    '''
    for t in range(num_teacher):
        c = solver.Constraint(0, 1)
        c.SetCoefficient(x[i][t], 1)
        c.SetCoefficient(x[j][t], 1)

for t in range(num_course):
    '''
    each course is taught by one teacher
    '''
    c = solver.Constraint(1, 1)
    for i in range(num_teacher):
        c.SetCoefficient(x[t][i], 1)

for i in range(num_teacher):
    '''
    z >= number of courses taught by teacher i
    '''
    c = solver.Constraint(0, num_course)
    c.SetCoefficient(z, 1)
    for j in range(num_course):
        c.SetCoefficient(x[j][i], -1)

obj = solver.Objective()
obj.SetCoefficient(z, 1)
obj.SetMinimization()
status = solver.Solve()

if status == pywraplp.Solver.OPTIMAL:
    for i in range(num_teacher):
        for t in range(num_course):
            if x[t][i].solution_value() > 0:
                print('x[', t, ',', i, '] =', x[t][i].solution_value())
    print(int(z.solution_value()))