import numpy as np

# side_cut_1 =  np.loadtxt('brT/2_second_sidecut_T1.txt')*0.001

# side_cut_1 =  np.loadtxt('brT/2_second_sidecut_T1.txt')*0.001
# new_array = np.array_split(side_cut_1, 9)
# for line in new_array:
#     print line

file = open('brT/2_second_sidecut_T1.txt', 'r')
data = file.read()
layers = data.split('\n\n')
for (i, layer) in enumerate(layers):
    # print layer , '\n'
    x = np.array(layer)
    print x, '\n'
    # for line in layer:
    #     x = np.array(line)
    #     # array = np.fromstring(line, dtype=float, sep=' ')
    #     print x


    # myarray = np.fromstring(line, dtype=float, sep=',')
    # print myarray
    # for points in line:
    #     pt = []
