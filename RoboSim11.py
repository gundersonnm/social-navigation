import math
import random

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString

# Assigns RGB color values
white = (255, 255, 255)
yellow = (255, 255, 102)
black = (0, 0, 0)
red = (213, 50, 80)
green = (0, 255, 0)
blue = (50, 153, 213)
gray = (131, 139, 139)

# Creation of nodes
N = 300

nodex = np.random.rand(N)
nodey = np.random.rand(N)
colors = (0,0,0)

# Define empty arrays for creation of obstacles
xArray = []
yArray = []
xminArray = []
yminArray = []
xmaxArray = []
ymaxArray = []

# Creation of obstacles
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
plt.xlim([0, 1.05])
plt.ylim([0, 1.05])
n=10
for i in range(0,n):
    x = random.uniform(0, 1)
    y = random.uniform(0, 1)
    ax.add_patch(matplotlib.patches.Rectangle((x, y),0.1,0.1,))
    xmin = x - 0.05
    ymin = y - 0.05
    xmax = x + 0.05
    ymax = y + 0.05
    xArray = np.append(xArray, x)
    yArray = np.append(yArray, y)
    xminArray = np.append(xminArray, xmin)
    yminArray = np.append(yminArray, ymin)
    xmaxArray = np.append(xmaxArray, xmax)
    ymaxArray = np.append(ymaxArray, ymax)

# Define arbitrary iterators and empty arrays
nodeDistArray = []
nodexArray = []
nodeyArray = []
minDisArray = []
minxArray = []
minyArray = []
currentNodeX = 0.5
currentNodeY = 0
nodexArray1 = []
nodeyArray1 = []
o = 0
pastCurrentNodeX = 0
pastCurrentNodeY = 0

# Adding the current node and final node to x and y arrays
nodexArray = np.append(nodex, currentNodeX)
nodeyArray = np.append(nodey, currentNodeY)

nodexArray = np.append(nodexArray, 0.5)
nodeyArray = np.append(nodeyArray, 1)

check1 = False

# Looping through multiple iterations of nodes to reach the final node
while o < 40:

    # Checking if its reached the final node
    if np.logical_and(currentNodeX == 0.5, currentNodeY == 1.0).any():
        check1 = True

    if check1 == True:
        break

    # Building arrays filled with distances from current nodes to nodes aroundd it
    nodexArray1 = []
    nodeyArray1 = []
    nodeDistArray = []
    p = 0
    x = 0
    for jj in nodexArray:

        if np.logical_and(nodexArray[x] != currentNodeX, nodeyArray[x] != currentNodeY).any():

            nodexArray1 = np.append(nodexArray1, nodexArray[x])
            nodeyArray1 = np.append(nodeyArray1, nodeyArray[x])
            nodeDistArray.append(math.sqrt((nodexArray1[p] - currentNodeX)**2 + (nodeyArray1[p] - currentNodeY)**2))

            allValuesArray = zip(nodeDistArray, nodexArray1, nodeyArray1)
            allValuesArray = list(allValuesArray)

            allValuesArray.sort()

            p = p + 1
        x = x + 1

    # Building arrays with the closest 10 nodes to current node
    minxArray = []
    minyArray = []
    minDisArray = []

    for dist in allValuesArray:
        minDisArray = np.append(minDisArray, dist[0])

    for numx in allValuesArray:
        minxArray = np.append(minxArray, numx[1])

    for numy in allValuesArray:
        minyArray = np.append(minyArray, numy[2])

    minDisArray = minDisArray[:10]
    minxArray = minxArray[:10]
    minyArray = minyArray[:10]

    l = 0
    checkcheck = False
    for kk in minxArray:
        if np.logical_and(kk == 0.5, minyArray[l] == 1.0).any():
            point1 = [currentNodeX, currentNodeY]
            point2 = [0.5, 1.0]
            x_values = [point1[0], point2[0]]
            y_values = [point1[1], point2[1]]
            ax.plot(x_values, y_values, '#76EE00')
            checkcheck = True
            break
        l = l + 1

    if checkcheck == True:
        break

    # Define arbitrary variable values
    r = 0
    minxArrayint = 0
    minyArrayint = 0
    finalNodeArray = []
    angleArray = []

    for nums in minDisArray:

        # assign min x and y to iterating values
        minxArrayint = minxArray[r]
        minyArrayint = minyArray[r]
        finalNodeArray.append(nums)

        # highlight min x y distance values values in blue
        ax.plot(minxArrayint, minyArrayint, 'bo')
        np.append(minxArrayint, currentNodeX)
        np.append(minyArrayint, currentNodeY)

        X,Y = np.meshgrid(minxArrayint, minyArrayint)
        positions = np.vstack([Y.ravel(), X.ravel()])
        origin = (currentNodeX, currentNodeY)
        for i in range(len(positions)):
            for j in range(len(positions[i])):
                x1 = positions[1][j]
                y1 = positions[0][j]
                line = LineString([origin, (x1, y1)])
                x2, y2 = line.xy
                ax.plot(0, 0, x2, y2, color = 'blue')

        # find value for angle between direct start to end line and each node point, save to array
        angleX = (minxArrayint - currentNodeX)
        angleY = (minyArrayint - currentNodeY)
        angle = math.degrees(math.atan(angleY / angleX))
        angleReal = 90 - abs(angle)
        angleArray = np.append(angleArray, angleReal)


        r = r + 1

    # Obstacle disqualifier!
    def intersects(s0,s1):
        dx0 = s0[1][0]-s0[0][0]
        dx1 = s1[1][0]-s1[0][0]
        dy0 = s0[1][1]-s0[0][1]
        dy1 = s1[1][1]-s1[0][1]
        p0 = dy1*(s1[1][0]-s0[0][0]) - dx1*(s1[1][1]-s0[0][1])
        p1 = dy1*(s1[1][0]-s0[1][0]) - dx1*(s1[1][1]-s0[1][1])
        p2 = dy0*(s0[1][0]-s1[0][0]) - dx0*(s0[1][1]-s1[0][1])
        p3 = dy0*(s0[1][0]-s1[1][0]) - dx0*(s0[1][1]-s1[1][1])
        return (p0*p1<=0) & (p2*p3<=0)


    n = 0
    b = 0
    intersectCounter = 0
    minxArray4 = []
    minyArray4 = []
    angleArray2 = []

    while b < 10:

        while n < 10:
            nodePath = [(currentNodeX, currentNodeY), (minxArray[n], minyArray[n])]
            side1 = [(xminArray[b], yminArray[b]), (xmaxArray[b], yminArray[b])]
            side2 = [(xminArray[b], yminArray[b]), (xminArray[b], ymaxArray[b])]
            side3 = [(xminArray[b], ymaxArray[b]), (xmaxArray[b], ymaxArray[b])]
            side4 = [(xmaxArray[b], ymaxArray[b]), (xmaxArray[b], yminArray[b])]

            if intersects(nodePath, side1) == True:
                intersectCounter = intersectCounter + 1

            if intersects(nodePath, side2) == True:
                intersectCounter = intersectCounter + 1

            if intersects(nodePath, side3) == True:
                intersectCounter = intersectCounter + 1

            if intersects(nodePath, side4) == True:
                intersectCounter = intersectCounter + 1

            if intersectCounter== 0:
                minxArray4 = np.append(minxArray4, minxArray[n])
                minyArray4 = np.append(minyArray4, minyArray[n])
                angleArray2 = np.append(angleArray2, angleArray[n])

            n = n + 1

        b = b + 1



    finalBehindIterator = 0
    j = 0
    finalAngleArray1 = []
    finalxArray = []
    finalyArray = []
    minyArray2 = 0
    minxArray2 = 0
    angleIterator = 0
    checker2 = 0

    # check to see if node is behind start point, disqualify node if it is.
    for nums1 in minyArray4:
        if nums1 > currentNodeY:
            finalxArray = np.append(finalxArray, minxArray4[finalBehindIterator])
            finalyArray = np.append(finalyArray, minyArray4[finalBehindIterator])
            finalAngleArray1 = np.append(finalAngleArray1, angleArray2[finalBehindIterator])
            checker2 = checker2 + 1
        finalBehindIterator = finalBehindIterator + 1
        continue

    # define arbitrary variables for angle disqualification
    finalAngleIterator = 0
    totalFinalAngleArray = []
    minxArray3 = []
    minyArray3 = []
    finalxArray1= []
    finalyArray1 = []
    checker3 = 0

    # Angle disqualifier!
    # check to see if angle between start to finish line to node is larger than 45 degrees
    # disqualify node if it is.
    for angles in finalAngleArray1:
        if angles < 45:
            minxArray3 = finalxArray[finalAngleIterator]
            minyArray3 = finalyArray[finalAngleIterator]
            finalxArray1 = np.append(finalxArray1, minxArray3)
            finalyArray1 = np.append(finalyArray1, minyArray3)
            totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
            checker3 = checker3 + 1
        finalAngleIterator = finalAngleIterator + 1

    if checker3 == 0 or checker2 == 0:
        if pastCurrentNodeX == minxArray[0] and pastCurrentNodeY == minyArray[0]:
            totalFinalAngleArray = np.append(totalFinalAngleArray, angleArray[1])
            finalxArray1 = np.append(finalxArray1, minxArray[1])
            finalyArray1 = np.append(finalyArray1, minyArray[1])
        else:
            totalFinalAngleArray = np.append(totalFinalAngleArray, angleArray[0])
            finalxArray1 = np.append(finalxArray, minxArray[0])
            finalyArray1 = np.append(finalyArray, minyArray[0])

    # define arbitrary variables for final chosen node desicion making
    finalDistanceIterator = 0
    b = 0
    minxArrayint1 = 0
    minyArrayint1 = 0
    finalfinalxArray = []
    finalfinalyArray = []
    finalfinalArray = []

    # check to see if angle between start and end point and node is smaller than 25 degrees
    # if it is, add it to a final qualifying array
    # choose the longest (last) node in array
    checker = False
    for num2 in totalFinalAngleArray:
        if num2 <= 25:
            finalfinalArray = np.append(finalfinalArray, num2)
            finalfinalxArray = np.append(finalfinalxArray, finalxArray1[b])
            finalfinalyArray = np.append(finalfinalyArray, finalyArray1[b])
            checker = True
        b = b + 1

    if np.logical_and(checker == False, len(finalAngleArray1) != 0).any():
        finalfinalArray = np.append(finalfinalArray, finalAngleArray1[0])
        finalfinalxArray = np.append(finalfinalxArray, finalxArray1[0])
        finalfinalyArray = np.append(finalfinalyArray, finalyArray1[0])

    if np.logical_and(checker == False, len(finalAngleArray1) == 0).any():
        finalfinalArray = np.append(finalfinalArray, angleArray2[0])
        finalfinalxArray = np.append(finalfinalxArray, minxArray4[0])
        finalfinalyArray = np.append(finalfinalyArray, minyArray4[0])

    # choose the last element in the x and y coordinates to draw a green line
    # line goes from start point to next
    xcoord = finalfinalxArray[-1]
    ycoord = finalfinalyArray[-1]
    pastCurrentNodeX = currentNodeX
    pastCurrentNodeY = currentNodeY
    point3 = [pastCurrentNodeX, pastCurrentNodeY]
    point4 = [xcoord, ycoord]
    x_values3 = [point3[0], point4[0]]
    y_values3 = [point3[1], point4[1]]
    ax.plot(x_values3, y_values3, '#76EE00')
    currentNodeX = xcoord
    currentNodeY = ycoord

    o = o + 1

# Plotting nodes and the start and end nodes
plt.scatter(nodex, nodey, c = colors)
start = plt.plot(0.5,0,'go')
end = plt.plot(0.5,1,'ro')
plt.show()
