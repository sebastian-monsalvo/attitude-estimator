# import vpython
# from vpython import box, sphere
from vpython import *

Xarrow = arrow(axis = vector(1, 0, 0), length = 0.05, shaftwidth = 0.001, color = color.red)
Yarrow = arrow(axis = vector(0, 1, 0), length = 0.05, shaftwidth = 0.001, color = color.green)
Zarrow = arrow(axis = vector(0, 0, 1), length = 0.05, shaftwidth = 0.001, color = color.blue)
Arduino = box(length = 0.05, width = 0.001, height = 0.1, color = vector(53.0, 137.0, 171.0)/256.0)
Arduino.pos = vector(0, 0, -0.01)

# tutorial
# x = box(length=0.1, width = 0.05, height = 0.0001)
# y = sphere(radius = 0.01, color = color.red)
# z = cylinder(length = 0.05, radius = 0.005, color = color.red)

# c = 0.001
# increasing = True
# while(True):
#     rate(100)
#     if increasing:
#         z.length = z.length + c
#         if z.length >= 0.05:
#             increasing = False
#     if not increasing:
#         z.length = z.length - c
#         if z.length <= 0.001:
#             increasing = True
