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

start_time = time.time()


# Assigns RGB color values to be used in simulation
white = (255, 255, 255)
yellow = (255, 255, 102)
black = (0, 0, 0)
red = (213, 50, 80)
green = (0, 255, 0)
blue = (50, 153, 213)
gray = (131, 139, 139)
colors = (0,0,0)

# Creation of nodes
gridX, gridY = np.mgrid[0:13, 0:10.1]

# Define empty arrays to hold the x and y coordinate values of each node
nodex = []
nodey = []

# Fill empty arrays with x and y coordinate values of each node
for nodes in gridX:
    nodex.append(nodes)
    nodex.append(nodes + 0.5)

for nodes1 in gridY:
    nodey.append(nodes1)
    nodey.append(nodes1 + 0.5)

# Define empty arrays for creation of obstacles
xObstacleArray = []
yObstacleArray = []
xminOsbtacleArray = []
yminObstacleArray = []
xmaxObstacleArray = []
ymaxObstacleArray = []

# Creation of obstacles, and adding their coordinate values to xmin, ymin, xmax, and ymax Arrays. We use these values to calculate intersection points.
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
plt.xlim([0, 10.5])
plt.ylim([0, 10.5])
n=20
for i in range(0,n):
    x = random.uniform(0, 9)
    y = random.uniform(0, 9)
    ax.add_patch(matplotlib.patches.Rectangle((x, y),1,1,))
    xmin = x
    ymin = y
    xmax = x + 1
    ymax = y + 1
    xObstacleArray = np.append(xObstacleArray, x)
    yObstacleArray = np.append(yObstacleArray, y)
    xminOsbtacleArray = np.append(xminOsbtacleArray, xmin)
    yminObstacleArray = np.append(yminObstacleArray, ymin)
    xmaxObstacleArray = np.append(xmaxObstacleArray, xmax)
    ymaxObstacleArray = np.append(ymaxObstacleArray, ymax)

# Define arbitrary iterators and empty arrays to hold the values of the 10 closest nodes to the CurrentNode
nodeDistArray = []
minDisArray = []

nodexArray = []
nodeyArray = []


currentNodeX = 5
currentNodeY = 0
o = 0
pastCurrentNodeX = 0
pastCurrentNodeY = 0

pastCurrentNodeXArray = []
pastCurrentNodeYArray = []

# Adding the initial node and final node to x and y arrays
nodexArray = np.append(nodex, currentNodeX)
nodeyArray = np.append(nodey, currentNodeY)

finalNodex = 5
finalNodey = 10

nodexArray = np.append(nodexArray, finalNodex)
nodeyArray = np.append(nodeyArray, finalNodey)
check1 = False

c = 0

# Loop encapsuling ALL path planning code. It will repeat until it either reaches the final node or has iterated 40 times (a failsafe for the code getting stuck somewhere).
while o < 40:
    condition1 = currentNodeX == finalNodex
    condition2 = currentNodeY == finalNodey
    # This checks if the path planner has reached the final node, and if it has, breaks the all encapsuling loop.
    if np.logical_and(condition1, condition2).any():
        print("its there!")
        check1 = True

    if check1 == True:
        break

    # Initialises arrays and fills them with the x values, y values, and distance values of all 100 nodes from the current node.
    nodeDistArray = []
    p = 0
    x = 0

    # Minimum values will only be added to the arrays if and only if the nodes are not current node X or Y, or any past current nodes.
    for values in nodexArray:
        condition1 = nodexArray[x] != currentNodeX
        condition2 =  nodeyArray[x] != currentNodeY
        if np.logical_and(condition1, condition2).any():

            if o == 0:
                nodeDistArray.append(math.sqrt((nodexArray[p] - currentNodeX)**2 + (nodeyArray[p] - currentNodeY)**2))

                allValuesArray = zip(nodeDistArray, nodexArray, nodeyArray)
                allValuesArray = list(allValuesArray)

                allValuesArray.sort()

                p = p + 1


            if o != 0:
                condition1 = nodexArray[x] != pastCurrentNodeX
                condition2 = nodeyArray[x] != pastCurrentNodeY
                if np.logical_and(condition1, condition2).any():

                    condition1 = nodexArray[x] != pastCurrentNodeXArray[o-2]
                    condition2 = nodeyArray[x] != pastCurrentNodeYArray[o-2]

                    if np.logical_and(condition1, condition2).any():

                        nodeDistArray.append(math.sqrt((nodexArray[p] - (currentNodeX))**2 + (nodeyArray[p] - currentNodeY)**2))
                        allValuesArray = zip(nodeDistArray, nodexArray, nodeyArray)
                        allValuesArray = list(allValuesArray)

                        allValuesArray.sort()

                        p = p + 1

        x = x + 1

    # Building arrays with the closest 10 nodes to current node
    minxArray = []
    minyArray = []
    minDisArray = []

    # adding the distances of the nodes with the 10 smallest distances to new arrays.
    for dist in allValuesArray:
        minDisArray = np.append(minDisArray, dist[0])

    # adding the x values of the nodes with the 10 smallest distances to new arrays.
    for numx in allValuesArray:
        minxArray = np.append(minxArray, numx[1])

    # adding the y values of the nodes with the 10 smallest distances to new arrays.
    for numy in allValuesArray:
        minyArray = np.append(minyArray, numy[2])

    # By sorting our values, if we take the first 10 of all of these arrays, they will be the 10 smallest distances.
    minDisArray = minDisArray[1:11]
    minxArray = minxArray[1:11]
    minyArray = minyArray[1:11]



    # If the final node is in the minimumn node array established above, it will be automaticlly chosen and the all encapsuling node will be broken.
    l = 0
    checkcheck = False
    for kk in minxArray:
        condition1 = kk == finalNodex
        condition2 = minyArray[l] == finalNodey
        if np.logical_and(condition1, condition2).any():
            point1 = [currentNodeX, currentNodeY]
            point2 = [finalNodex, finalNodey]
            x_values = [point1[0], point2[0]]
            y_values = [point1[1], point2[1]]
            ax.plot(x_values, y_values, '#76EE00')
            checkcheck = True
            break
        l = l + 1

    if checkcheck == True:
        break

    # Define arbitrary variable values for the drawing of blue lines and calculating of angle values.
    r = 0
    minxArrayint = 0
    minyArrayint = 0
    finalNodeArray = []
    angleArray = []

    # This loop both creates blue lines that go from the current node to each of the 10 closest nodes, and calculates the angle between each of those nodes and the current node.
    for nums in minDisArray:

        # assign min x and y to iterating values
        minxArrayint = minxArray[r]
        minyArrayint = minyArray[r]
        finalNodeArray.append(nums)
        angleReal = 0
        angleTest = 0

        # highlight min x y distance values values in blue
        ax.plot(minxArrayint, minyArrayint, 'bo')

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
        # if the node coordinate is less than both currentNodeX and currentNodeY, it will be subtracted from 180.
        condition1 = minxArrayint < currentNodeX
        condition2 = minyArrayint < currentNodeY
        if np.logical_and(condition1, condition2).any():

            angleTest = math.atan2((minyArrayint - currentNodeY), (minxArrayint - currentNodeX))
            degAngleTest = 360 - abs(math.degrees(angleTest))
            angleArray = np.append(angleArray, degAngleTest)

        # if the node x coordinate is larger than currentNodeX, but the y coordinate is less than currentNodeY, it will be subtracted from 360.
        condition1 = minxArrayint > currentNodeX
        condition2 = minyArrayint < currentNodeY
        if np.logical_and(condition1, condition2).any():

            angleTest = math.atan2((minyArrayint - currentNodeY), (minxArrayint - currentNodeX))
            degAngleTest = 360 - abs(math.degrees(angleTest))
            angleArray = np.append(angleArray, degAngleTest)


        # if the node coordinate is directly above or below the current node, its angle will be set to 90 or 270 degrees automaticlly.

        if minxArrayint == currentNodeX:

            if minyArrayint < currentNodeY:
                angleReal = 270
                angleArray = np.append(angleArray, angleReal)
                #print(angleReal)

            if minyArrayint > currentNodeY:
                angleReal = 90
                angleArray = np.append(angleArray, angleReal)
                #print(angleReal)


        # if the node does not fit any of those conditions, it will follow the typical angle finding process without any abnormal subtraction
        else:

            angleTest = math.atan2((minyArrayint - currentNodeY), (minxArrayint - currentNodeX))
            degAngleTest = math.degrees(angleTest)


            if degAngleTest < 0:
                degAngleTest = 360 - abs(math.degrees(angleTest))

            else:
                angleArray = np.append(angleArray, degAngleTest)

        r = r + 1


    print("all angles", angleArray)
    print("all x values", minxArray)
    print("all y values", minyArray)



    # Obstacle disqualifier! The obstacle disqualifier works by checking if each node line intersects with any side of the square obstacles. If there is any intersection  detected, the node will be disqualified.
    def intersects(s0,s1):
        # Sets the sides of the obstacle equal to dx0, dx1... and checks if they intersect with any node lines.
        dx0 = s0[1][0]-s0[0][0]
        dx1 = s1[1][0]-s1[0][0]
        dy0 = s0[1][1]-s0[0][1]
        dy1 = s1[1][1]-s1[0][1]
        p0 = dy1*(s1[1][0]-s0[0][0]) - dx1*(s1[1][1]-s0[0][1])
        p1 = dy1*(s1[1][0]-s0[1][0]) - dx1*(s1[1][1]-s0[1][1])
        p2 = dy0*(s0[1][0]-s1[0][0]) - dx0*(s0[1][1]-s1[0][1])
        p3 = dy0*(s0[1][0]-s1[1][0]) - dx0*(s0[1][1]-s1[1][1])
        return (p0*p1<=0) & (p2*p3<=0)

    # Initialises variables and iterators for counting intersections and creating arrays for node lines without intersections.
    n = 0
    minxArray4 = []
    minyArray4 = []
    angleArray2 = []

    # iterates through all 10 NODES (n)
    while n < 9:
        # if the distance from current node to a corner of an obstacle is greater than 4, dont check it for collisions
        b = 0
        intersectCounter = 0

        # iterates through all 20 BLOCKS (b)
        while b < 20:
            nodeDistance = 0
            #distance calculation from node to edge of obstacle
            nodeDistance = math.sqrt((xminOsbtacleArray[b] - currentNodeX)**2 + (yminObstacleArray[b] - currentNodeY)**2)
            if nodeDistance <= 4:
                nodePath = [(currentNodeX, currentNodeY), (minxArray[n], minyArray[n])]
                side1 = [(xminOsbtacleArray[b], yminObstacleArray[b]), (xmaxObstacleArray[b], yminObstacleArray[b])]
                side2 = [(xminOsbtacleArray[b], yminObstacleArray[b]), (xminOsbtacleArray[b], ymaxObstacleArray[b])]
                side3 = [(xminOsbtacleArray[b], ymaxObstacleArray[b]), (xmaxObstacleArray[b], ymaxObstacleArray[b])]
                side4 = [(xmaxObstacleArray[b], ymaxObstacleArray[b]), (xmaxObstacleArray[b], yminObstacleArray[b])]

                # checks if any of the 4 sides intersect with each iterating node line or blockby using intersects function.
                if intersects(nodePath, side1):
                    intersectCounter = intersectCounter + 1

                if intersects(nodePath, side2):
                    intersectCounter = intersectCounter + 1

                if intersects(nodePath, side3):
                    intersectCounter = intersectCounter + 1

                if intersects(nodePath, side4):
                    intersectCounter = intersectCounter + 1


            b = b + 1


        # If there were no intersections for a node, it will be added to an array to be analyzed further.
        if intersectCounter == 0:
            minxArray4 = np.append(minxArray4, minxArray[n])
            minyArray4 = np.append(minyArray4, minyArray[n])
            angleArray2 = np.append(angleArray2, angleArray[n])

        n = n + 1

    print("after obstacle", angleArray2)

    # Initializing iterators and arrays for checking if there are any nodes behind the current node
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

    for angles in angleArray2:
        condition1 = angles >= 0
        condition2 = angles <= 180
        if np.logical_and(condition1, condition2):
            finalxArray = np.append(finalxArray, minxArray4[finalBehindIterator])
            finalyArray = np.append(finalyArray, minyArray4[finalBehindIterator])
            finalAngleArray1 = np.append(finalAngleArray1, angleArray2[finalBehindIterator])
            checker2 = checker2 + 1
        finalBehindIterator = finalBehindIterator + 1
        # continue

    print("after behind", finalAngleArray1)

    # define arbitrary variables for angle disqualification
    finalAngleIterator = 0
    totalFinalAngleArray = []
    minxArray3 = []
    minyArray3 = []
    finalxArray1= []
    finalyArray1 = []
    backupAngleArray = []
    backupxArray = []
    backupyArray = []
    checker3 = 0
    bestfinalxArray1 = []
    bestfinalyArray1 = []
    besttotalFinalAngleArray = []
    backtrackIterator = 0

    # Angle disqualifier! This will check to see if angle between start to finish line to node is larger than 45 degrees, and do not add the node to the new array if it is.
    for angles in finalAngleArray1:

        condition1 = finalyArray[backtrackIterator] != pastCurrentNodeY
        condition2 = finalxArray[backtrackIterator] != pastCurrentNodeX
        if np.logical_or(condition1, condition2).any():

            if len(pastCurrentNodeXArray)>=2:

                condition1 = finalyArray[backtrackIterator] != pastCurrentNodeYArray[-2]
                condition2 = finalxArray[backtrackIterator] != pastCurrentNodeXArray[-2]
                if np.logical_or(condition1, condition2).any():

                    if currentNodeX < finalNodex:

                        condition1 = angles <= 90
                        condition2 = angles >= 0
                        if np.logical_and(condition1, condition2).any():
                            #minxArray3 = finalxArray[finalAngleIterator]
                            #minyArray3 = finalyArray[finalAngleIterator]
                            finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                            finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                            totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                            checker3 = checker3 + 1

                        condition1 = angles >= 90
                        condition2 = angles <= 180
                        if np.logical_and(condition1, condition2).any():
                            backupAngleArray = np.append(backupAngleArray, angles)
                            backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                            backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
                            # add to backup array

                    if currentNodeX > finalNodex:

                        condition1 = angles >= 90
                        condition2 = angles <= 180
                        if np.logical_and(condition1, condition2).any():
                            #minxArray3 = finalxArray[finalAngleIterator]
                            #minyArray3 = finalyArray[finalAngleIterator]
                            finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                            finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                            totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                            checker3 = checker3 + 1

                        condition1 = angles <= 90
                        condition2 = angles >= 0
                        if np.logical_or(condition1, condition2):
                            backupAngleArray = np.append(backupAngleArray, angles)
                            backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                            backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
                        # add to backup array

                    if angles == 90:
                        bestfinalxArray1 = np.append(bestfinalxArray1, finalxArray[finalAngleIterator])
                        bestfinalyArray1 = np.append(bestfinalyArray1, finalyArray[finalAngleIterator])
                        besttotalFinalAngleArray = np.append(besttotalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                        checker3 = checker3 + 1

                    else:
                         backupAngleArray = np.append(backupAngleArray, angles)
                         backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                         backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])

            else:

                if currentNodeX < finalNodex:
                    condition1 = angles <= 90
                    condition2 = angles >= 0
                    if np.logical_and(condition1, condition2).any():
                        #minxArray3 = finalxArray[finalAngleIterator]
                        #minyArray3 = finalyArray[finalAngleIterator]
                        finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                        finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                        totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                        checker3 = checker3 + 1

                    condition1 = angles >= 90
                    condition2 = angles <= 180
                    if np.logical_and(condition1, condition2).any():
                        backupAngleArray = np.append(backupAngleArray, angles)
                        backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                        backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
                        # add to backup array

                if currentNodeX > finalNodex:

                    condition1 = angles >= 90
                    condition2 = angles <= 180
                    if np.logical_and(condition1, condition2).any():
                        #minxArray3 = finalxArray[finalAngleIterator]
                        #minyArray3 = finalyArray[finalAngleIterator]
                        finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                        finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                        totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                        checker3 = checker3 + 1

                    condition1 = angles <= 90
                    condition2 = angles >= 0
                    if np.logical_or(condition1, condition2):
                        backupAngleArray = np.append(backupAngleArray, angles)
                        backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                        backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
                    # add to backup array

                if angles == 90:
                    bestfinalxArray1 = np.append(bestfinalxArray1, finalxArray[finalAngleIterator])
                    bestfinalyArray1 = np.append(bestfinalyArray1, finalyArray[finalAngleIterator])
                    besttotalFinalAngleArray = np.append(besttotalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                    checker3 = checker3 + 1

                else:
                     backupAngleArray = np.append(backupAngleArray, angles)
                     backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                     backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])


        finalAngleIterator = finalAngleIterator + 1

        backtrackIterator = backtrackIterator + 1

    backupBackupAngleArray = []
    backupBackupfinalxArray1 = []
    backupBackupfinalyArray1 = []

    # If there are no nodes left after the 'behind' disqualification (and consequential angle disqualification), the program will choose a node that is over 45 degrees / behind the current node.
    backupBackupAngleArray = np.append(totalFinalAngleArray, angleArray2[0])
    backupBackupfinalxArray1 = np.append(finalxArray1, minxArray4[0])
    backupBackupfinalyArray1 = np.append(finalyArray1, minyArray4[0])

    print("moving straight", besttotalFinalAngleArray)
    print("drifting", totalFinalAngleArray)
    print("no drift", backupAngleArray)
    print("move backwards", backupBackupAngleArray)

#order of choosing: straight (bestfinalxArray1, besttotalFinalAngleArray), drifting (finalxArray1, totalFinalAngleArray), no drift (backupxArray, backupAngleArray), move backwards new (backupBackupfinalxArray1, backupBackupAngleArray), move backwards repeat (pastCurrentNodeX, backupAngleArray)

    # Define arbitrary variables for final chosen node desicion making
    finalDistanceIterator = 0
    b = 0
    minxArrayint1 = 0
    minyArrayint1 = 0
    finalfinalxArray = []
    finalfinalyArray = []
    finalfinalArray = []
    backupFinalAngleArray = []
    backupFinalxArray = []
    backupFinalyArray = []
    choosingxArray = []
    choosingyArray = []
    choosingAngleArray = []

    checker1 = False

    if len(bestfinalxArray1) != 0:
        choosingxArray = np.append(choosingxArray, bestfinalxArray1[-1])
        choosingyArray = np.append(choosingyArray, bestfinalyArray1[-1])
        choosingAngleArray = np.append(choosingAngleArray, besttotalFinalAngleArray[-1])
        print("move straight")
        checker1 = True

    if checker1 == False:
        if len(finalxArray1) != 0:
            choosingxArray = np.append(choosingxArray, finalxArray1[-1])
            choosingyArray = np.append(choosingyArray, finalyArray1[-1])
            choosingAngleArray = np.append(choosingAngleArray, totalFinalAngleArray[-1])
            print("drifting")
            checker1 = True

    if checker1 == False:
        if len(backupxArray) != 0:
            choosingxArray = np.append(choosingxArray, backupxArray[-1])
            choosingyArray = np.append(choosingyArray, backupyArray[-1])
            choosingAngleArray = np.append(choosingAngleArray, backupAngleArray[-1])
            print("no drift")
            checker1 = True

    if checker1 == False:
        if len(backupBackupfinalxArray1) != 0:
            choosingxArray = np.append(choosingxArray, backupBackupfinalxArray1[-1])
            choosingyArray = np.append(choosingyArray, backupBackupfinalyArray1[-1])
            choosingAngleArray = np.append(choosingAngleArray, backupBackupAngleArray[-1])
            print("move backward")
            checker1 = True

    if checker1 == False:
        choosingxArray = np.append(choosingxArray, pastCurrentNodeX)
        choosingyArray = np.append(choosingyArray, pastCurrentNodeY)
        choosingAngleArray = np.append(choosingAngleArray, 270)
        print("backtrack")


    # Choosing of the next node. The program chooses the last value of the qualifying nodes, to ensure the longest path is being chosen.
    # The previous current node will be set to the past current node and added to the past current node array.
    print("CHOSEN ANGLE", choosingAngleArray[-1])
    print("CHOSEN X", choosingxArray[-1])
    print("CHOSEN Y", choosingyArray[-1])

    xcoord = choosingxArray[-1]
    ycoord = choosingyArray[-1]
    pastCurrentNodeX = currentNodeX
    pastCurrentNodeY = currentNodeY
    point3 = [pastCurrentNodeX, pastCurrentNodeY]
    point4 = [xcoord, ycoord]
    x_values3 = [point3[0], point4[0]]
    y_values3 = [point3[1], point4[1]]
    ax.plot(x_values3, y_values3, '#76EE00')
    currentNodeX = xcoord
    currentNodeY = ycoord
    pastCurrentNodeXArray = np.append(pastCurrentNodeXArray, pastCurrentNodeX)
    pastCurrentNodeYArray = np.append(pastCurrentNodeYArray, pastCurrentNodeY)

# The new current node will have a green line drawn to it from the past current node.
    if o>1:
        x_values4 = [pastCurrentNodeXArray[-2], pastCurrentNodeXArray[-1]]
        y_values4 = [pastCurrentNodeYArray[-2], pastCurrentNodeYArray[-1]]
        ax.plot(x_values4, y_values4, '#76EE00')
    if o == 1:
        x_values4 = [5, pastCurrentNodeX]
        y_values4 = [0, pastCurrentNodeY]
        ax.plot(x_values4, y_values4, '#76EE00')


    o = o + 1
    c = c + 1


# Calculates how long the program took to run from start to finish.
print("My program took", time.time() - start_time, "to run")

# Plotting nodes and the start and end nodes
plt.scatter(nodex, nodey, c = colors)
start = plt.plot(5,0,'go')
end = plt.plot(finalNodex,finalNodey,'ro')
plt.show()
