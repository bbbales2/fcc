#%%

import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

basis = numpy.array([[0.0, 0.0, 0.0],
                    [1.0 / 2.0, 1.0 / 2.0, 0.0],
                    [0.0, 1.0 / 2.0, 1.0 / 2.0],
                    [1.0 / 2.0, 0.0, 1.0 / 2.0]])

ang1 = numpy.arctan(2.0 / numpy.sqrt(2.0))

distance = (numpy.sqrt(2.0) / 2.0) * numpy.sin(ang1) / numpy.sin(numpy.pi - ang1 - numpy.pi / 4.0)
print distance
d = numpy.sqrt(3.0) / 3

l111 = numpy.array((1.0, 1.0, 1.0))
l111 = l111 / numpy.linalg.norm(l111)

atoms = []

print sys.argv

if len(sys.argv) == 2 and sys.argv[1] == 'radius':
    R = 2
else:
    R = 1

for i in range(-R, R + 1):
    for j in range(-R, R + 1):
        for k in range(-R, R + 1):
            for l in range(3):
                for b in range(4):
                    append = False

                    loc = numpy.array((i, j, k)) + basis[b] + l111 * d * l

                    if len(sys.argv) < 2:
                        append = True
                    else:
                        if sys.argv[1] == 'cut' and numpy.dot(loc, l111) < 0.5:
                            append = True

                        if sys.argv[1] == 'radius' and numpy.linalg.norm(loc) < 1.0:
                            append = True

                    if append:
                        atoms.append((numpy.array((i, j, k)) + basis[b] + l111 * d * l, (i, j, k), l, b))



xs = [atom[0][0] for atom in atoms]
ys = [atom[0][1] for atom in atoms]
zs = [atom[0][2] for atom in atoms]
cs = [['r', 'g', 'b', 'r'][atom[2]] for atom in atoms]


fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')

ax.scatter(xs, ys, zs, c = cs)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()