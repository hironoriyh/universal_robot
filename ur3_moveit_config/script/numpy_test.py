import numpy as np

# side_cut_1 =  np.loadtxt('brT/2_second_sidecut_T1.txt')*0.001

side_cut_1 =  np.loadtxt('brT/2_second_sidecut_T1.txt')*0.001
new_arrays = np.array_split(side_cut_1, 35)
print np.shape(new_arrays)
for layer  in  new_arrays:
    # print len(pt)
    for pt in layer:
        print pt[2]

# print len(new_array)
#
# lines = []
# file = open('brT/2_second_sidecut_T1.txt', 'r')
# data = file.read()
# layers = data.split('\n\n')
# for (i, layer) in enumerate(layers):
#     print type(layer)
#     x = np.array(layer)
#     # print x, '\n'
#     print x[10]
#     # y = x.asfarray(np, float)
#     # lines.append(y)
# print lines
    # for line in layer:
    #     x = np.array(line)
    #     # array = np.fromstring(line, dtype=float, sep=' ')
    #     print x


    # myarray = np.fromstring(line, dtype=float, sep=',')
    # print myarray
    # for points in line:
    #     pt = []
