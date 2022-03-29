import matplotlib.pyplot as plt
import math
import random

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString
import time
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

#Equation1 = y1 - (a*(x1**2) - (b*x1) - c == 0
#Equation2 = y2 - (a*(x2**2) - (b*x2) - c == 0
#Equation3 = y3 - (a*(x3**2) - (b*x3) - c == 0


a=[]
b=[]
# y=0
# x=-50

import numpy as np

#### example from youtube https://www.youtube.com/watch?v=zPDp_ewoyhM

def jacobian_example(x,y):

    return [[1,2],[2*x,8*y]]

def function_example(x,y):

    return [(-1)*(x+(2*y)-2),(-1)*((x**2)+(4*(y**2))-4)]
####################################################################


### agora com os dados do exercício

def jacobian_exercise(x,y,z):

    return [[1,1,1],[2*x,2*y,2*z],[np.exp(x),x,-x]]

#print (jacobian_exercise(1,2,3))
jotinha  = (jacobian_exercise(1,2,3))

def function_exercise(x,y,z):

    return [(-1)*(x+y+z-3),(-1)*((x**2)+(y**2)+(z**2)-5),(-1)*((np.exp(x))+(x*y)-(x*z)-1)]

#print (function_exercise(1,2,3))
bezao = (function_exercise(1,2,3))

def x_delta_by_gauss(J,b):

    return np.linalg.solve(J,b)

print (x_delta_by_gauss(jotinha, bezao))
x_delta_test = x_delta_by_gauss(jotinha,bezao)

def x_plus_1(x_delta,x_previous):

    x_next = x_previous + x_delta

    return x_next

print (x_plus_1(x_delta_test,[1,2,3]))

def newton_method(x_init):

    first = x_init[0]

    second = x_init[1]

    third = x_init[2]

    jacobian = jacobian_exercise(first, second, third)

    vector_b_f_output = function_exercise(first, second, third)

    x_delta = x_delta_by_gauss(jacobian, vector_b_f_output)

    x_plus_1 = x_delta + x_init

    return x_plus_1

def iterative_newton(x_init):

    counter = 0

    x_old = x_init
   #print ("x_old", x_old)

    x_new = newton_method(x_old)
   #print ("x_new", x_new)

    diff = np.linalg.norm(x_old-x_new)
   #print (diff)

    while diff>0.0000000000001:

        counter += 1

       #print ("x_old", x_old)
        x_new = newton_method(x_old)
       #print ("x_new", x_new)

        diff = np.linalg.norm(x_old-x_new)
       #print (diff)

        x_old = x_new

    convergent_val = x_new
   #print (counter)

    return convergent_val

#print (iterative_newton([1,2]))
#print (list(map(float,(iterative_newton([100,200,3])))))


for x in range(-50,50,1):
    y=x**2+2*x+1
    #print(y)
    a.append(x)
    b.append(y)
    #x= x+1

fig= plt.figure()
axes=fig.add_subplot(111)
axes.plot(a,b)
plt.show()
