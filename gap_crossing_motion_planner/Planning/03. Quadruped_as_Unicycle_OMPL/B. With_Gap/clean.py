import numpy as np


def aggregate(c, s, Controls_list, Steps_list):
    Steps_list[-1] = Steps_list[-1] - s
    if Steps_list[-1] < 0:
        Controls_list[-1] = c
        Steps_list[-1] = -Steps_list[-1]
    elif Steps_list[-1] == 0:
        Controls_list.pop()
        Steps_list.pop()


data = np.loadtxt('path_example.txt')

dim = len(data[:, 0])

Controls = data[:, 8:9]
Steps = data[:, 9:10]

Controls_list = [int(Controls[1])]
Steps_list  = [int(Steps[1])]

out_file = open('plan_example.txt','w')

for i in range(2, dim):
    c = int(Controls[i])
    s = int(Steps[i])
    if c == Controls_list[-1]:
        Steps_list[-1] = Steps_list[-1] + int(s)

    elif c == 0 and Controls_list[-1] == 1:
        aggregate(c, s, Controls_list, Steps_list)

    elif c == 1 and Controls_list[-1] == 0:
        aggregate(c, s, Controls_list, Steps_list)

    elif c == 2 and Controls_list[-1] == 3:
        aggregate(c, s, Controls_list, Steps_list)

    elif c == 3 and Controls_list[-1] == 2:
        aggregate(c, s, Controls_list, Steps_list)

    elif c == 4 and Controls_list[-1] == 5:
        aggregate(c, s, Controls_list, Steps_list)

    elif c == 5 and Controls_list[-1] == 4:
        aggregate(c, s, Controls_list, Steps_list)

    
    else:
        Controls_list.append(int(c))
        Steps_list.append(int(s))

for i in range(0, len(Controls_list)):
    out_file.write(str(Controls_list[i]) + " " + str(Steps_list[i]) + "\n")
    
out_file.close()
