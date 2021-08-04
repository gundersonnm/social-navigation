import math
import random

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString

white = (255, 255, 255)
yellow = (255, 255, 102)
black = (0, 0, 0)
red = (213, 50, 80)
green = (0, 255, 0)
blue = (50, 153, 213)
gray = (131, 139, 139)

N = 100

nodex = np.random.rand(N)
nodey = np.random.rand(N)
colors = (0,0,0)


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

nodexArray = np.append(nodex, currentNodeX)
nodeyArray = np.append(nodey, currentNodeY)

nodexArray = np.append(nodexArray, 0.5)
nodeyArray = np.append(nodeyArray, 1)

check1 = False

while o < 20:

    if np.logical_and(currentNodeX == 0.5, currentNodeY == 1.0).any():
        check1 = True

    if check1 == True:
        break

    nodexArray1 = []
    nodeyArray1 = []
    nodeDistArray = []
    p = 0
    x = 0
    for jj in nodexArray:

        if np.logical_and(nodexArray[x] != currentNodeX, nodeyArray[x] != currentNodeY).any():

            nodexArray1 = np.append(nodexArray1, nodexArray[x])
            nodeyArray1 = np.append(nodeyArray1, nodeyArray[x])
            #newx = nodexArray1[p]
            #newy = nodeyArray1[p]
            nodeDistArray.append(math.sqrt((nodexArray1[p] - currentNodeX)**2 + (nodeyArray1[p] - currentNodeY)**2))

            allValuesArray = zip(nodeDistArray, nodexArray1, nodeyArray1)
            allValuesArray = list(allValuesArray)

            allValuesArray.sort()

            p = p + 1
        x = x + 1

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
            #currentNodeX = kk
            #currentNodeY = minyArray[l]
            point1 = [currentNodeX, currentNodeY]
            point2 = [0.5, 1.0]
            x_values = [point1[0], point2[0]]
            y_values = [point1[1], point2[1]]
            plt.plot(x_values, y_values, '#76EE00')
            checkcheck = True
            break
        l = l + 1

    if checkcheck == True:
        break

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
        plt.plot(minxArrayint, minyArrayint, 'bo')
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
                plt.plot(0, 0, x2, y2, color = 'blue')

        # find value for angle between direct start to end line and each node point, save to array
        angleX = (minxArrayint - currentNodeX)
        angleY = (minyArrayint - currentNodeY)
        angle = math.degrees(math.atan(angleY / angleX))
        angleReal = 90 - abs(angle)
        angleArray = np.append(angleArray, angleReal)


        r = r + 1
    #print(minxArray)
    #print(minyArray)
    print(angleArray)
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
    for nums1 in minyArray:
        if nums1 > currentNodeY:
            finalxArray = np.append(finalxArray, minxArray[finalBehindIterator])
            finalyArray = np.append(finalyArray, minyArray[finalBehindIterator])
            finalAngleArray1 = np.append(finalAngleArray1, angleArray[finalBehindIterator])
            #finalAngleArray1.append(angleArray[finalBehindIterator])
            #finalxArray.append(minxArray[finalBehindIterator])
            #finalyArray.append(minyArray[finalBehindIterator])
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

    # check to see if angle between start to finish line to node is larger than 45 degrees
    # disqualify node if it is.
    for angles in finalAngleArray1:
        if angles < 45:
            minxArray3 = finalxArray[finalAngleIterator]
            minyArray3 = finalyArray[finalAngleIterator]
            finalxArray1 = np.append(finalxArray1, minxArray3)
            finalyArray1 = np.append(finalyArray1, minyArray3)
            totalFinalAngleArray = np.append(totalFinalAngleArray, angles)
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

    print(totalFinalAngleArray)
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

    if checker == False:
        finalfinalArray = np.append(finalfinalArray, totalFinalAngleArray[0])
        finalfinalxArray = np.append(finalfinalxArray, finalxArray1[0])
        finalfinalyArray = np.append(finalfinalyArray, finalyArray1[0])


    xcoord = finalfinalxArray[-1]
    ycoord = finalfinalyArray[-1]
    pastCurrentNodeX = currentNodeX
    pastCurrentNodeY = currentNodeY
    point3 = [pastCurrentNodeX, pastCurrentNodeY]
    point4 = [xcoord, ycoord]
    x_values3 = [point3[0], point4[0]]
    y_values3 = [point3[1], point4[1]]
    plt.plot(x_values3, y_values3, '#76EE00')
    currentNodeX = xcoord
    currentNodeY = ycoord


    #point1 = [currentNodeX, currentNodeY]
    #point2 = [0.5, 1.0]
    #x_values = [point1[0], point2[0]]
    #y_values = [point1[1], point2[1]]
    #plt.plot(x_values, y_values, 'blue')
    o = o + 1
plt.scatter(nodex, nodey, c = colors)
start = plt.plot(0.5,0,'go')
end = plt.plot(0.5,1,'ro')
plt.show()
