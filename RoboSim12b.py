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

dynamicxArray = []
dynamicyArray = []

# set animation to iterate one frame with each new node chosen (frame == o?), with each new iteration save iterating center value to obstacle arrray. navigate around that if applicable.
 
#fig = plt.figure()
#fig.set_dpi(100)
#fig.set_size_inches(7, 6.5)

#ax = plt.axes(xlim=(0, 10), ylim=(0, 10))
#patch = plt.Circle((0, 0), 0.5, fc='r')

#def init():
#    patch.center = (5, 5)
#    ax.add_patch(patch)
#    return patch,

#def animate(i):
#    x, y = patch.center
#    x = i
#    y = i
#    patch.center = (x, y)
#    return patch,

#anim = animation.FuncAnimation(fig, animate,
#   init_func=init,
 #  frames=500,
#   interval=200,
#   blit=True)

#i = 0
#while i < 10:
#    dynamicxArray = np.append(dynamicxArray, i)
#    dynamicyArray = np.append(dynamicyArray, (i))
#    i = i + 1

print(dynamicxArray, dynamicyArray)

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
gridX, gridY = np.mgrid[0:11, 0:11]

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
xArray = []
yArray = []
xminArray = []
yminArray = []
xmaxArray = []
ymaxArray = []

# Creation of obstacles, and adding their coordinate values to xmin, ymin, xmax, and ymax Arrays. We use these values to calculate intersection points.
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
plt.xlim([0, 10.05])
plt.ylim([0, 10.05])
n=20
for i in range(0,n):
    x = random.uniform(0, 9)
    y = random.uniform(0, 9)
    ax.add_patch(matplotlib.patches.Rectangle((x, y),1,1,))
    xmin = x
    ymin = y
    xmax = x + 1
    ymax = y + 1
    xArray = np.append(xArray, x)
    yArray = np.append(yArray, y)
    xminArray = np.append(xminArray, xmin)
    yminArray = np.append(yminArray, ymin)
    xmaxArray = np.append(xmaxArray, xmax)
    ymaxArray = np.append(ymaxArray, ymax)

# Define arbitrary iterators and empty arrays to hold the values of the 10 closest nodes to the CurrentNode
nodeDistArray = []
nodexArray = []
nodeyArray = []
minDisArray = []
minxArray = []
minyArray = []
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

nodexArray = np.append(nodexArray, 5)
nodeyArray = np.append(nodeyArray, 10)
check1 = False

c = 0

# Loop encapsuling ALL path planning code. It will repeat until it either reaches the final node or has iterated 40 times (a failsafe for the code getting stuck somewhere).
while o < 40:

    # This checks if the path planner has reached the final node, and if it has, breaks the all encapsuling loop.
    if np.logical_and(currentNodeX == 5, currentNodeY == 10).any():
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

        if np.logical_and(nodexArray[x] != currentNodeX, nodeyArray[x] != currentNodeY).any():

            if o == 0:
                nodeDistArray.append(math.sqrt((nodexArray[p] - currentNodeX)**2 + (nodeyArray[p] - currentNodeY)**2))

                allValuesArray = zip(nodeDistArray, nodexArray, nodeyArray)
                allValuesArray = list(allValuesArray)

                allValuesArray.sort()

                p = p + 1


            if o != 0:
                if np.logical_and(nodexArray[x] != pastCurrentNodeX, nodeyArray[x] != pastCurrentNodeY).any():

                    if np.logical_and(nodexArray[x] != pastCurrentNodeXArray[o-2], nodeyArray[x] != pastCurrentNodeYArray[o-2]).any():

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
        if np.logical_and(kk == 5, minyArray[l] == 10).any():
            point1 = [currentNodeX, currentNodeY]
            point2 = [5, 10]
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
        if np.logical_and(minxArrayint < currentNodeX, minyArrayint < currentNodeY).any():

            angleTest = math.atan2((minyArrayint - currentNodeY), (minxArrayint - currentNodeX))
            degAngleTest = 360 - abs(math.degrees(angleTest))
            angleArray = np.append(angleArray, degAngleTest)

        # if the node x coordinate is larger than currentNodeX, but the y coordinate is less than currentNodeY, it will be subtracted from 360.
        if np.logical_and(minxArrayint > currentNodeX, minyArrayint < currentNodeY).any():

            angleTest = math.atan2((minyArrayint - currentNodeY), (minxArrayint - currentNodeX))
            degAngleTest = 360 - abs(math.degrees(angleTest))
            angleArray = np.append(angleArray, degAngleTest)


        # if the node coordinate is directly above or below the current node, its angle will be set to 90 or 270 degrees automaticlly.
        print("minxArrayint", minxArrayint)

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
        b = 0
        intersectCounter = 0

        # iterates through all 20 BLOCKS (b)
        while b < 20:
            nodePath = [(currentNodeX, currentNodeY), (minxArray[n], minyArray[n])]
            side1 = [(xminArray[b], yminArray[b]), (xmaxArray[b], yminArray[b])]
            side2 = [(xminArray[b], yminArray[b]), (xminArray[b], ymaxArray[b])]
            side3 = [(xminArray[b], ymaxArray[b]), (xmaxArray[b], ymaxArray[b])]
            side4 = [(xmaxArray[b], ymaxArray[b]), (xmaxArray[b], yminArray[b])]

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
        if np.logical_and(angles >= 0, angles <= 180):
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

    # Angle disqualifier! This will check to see if angle between start to finish line to node is larger than 45 degrees, and do not add the node to the new array if it is.
    for angles in finalAngleArray1:

        if currentNodeX < 5:
            print("less than 5")
            if np.logical_and(angles <= 90, angles >= 0).any():
                #minxArray3 = finalxArray[finalAngleIterator]
                #minyArray3 = finalyArray[finalAngleIterator]
                finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                checker3 = checker3 + 1


            if np.logical_and(angles >= 90, angles <= 180).any():
                print("doodoo")
                backupAngleArray = np.append(backupAngleArray, angles)
                backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
                # add to backup array

        if currentNodeX > 5:
            print("more than 5")
            if np.logical_and(angles >= 90, angles <= 180).any():
                #minxArray3 = finalxArray[finalAngleIterator]
                #minyArray3 = finalyArray[finalAngleIterator]
                finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                checker3 = checker3 + 1

            if np.logical_or(angles <= 90, angles >= 180):
                print("doodoo2")
                backupAngleArray = np.append(backupAngleArray, angles)
                backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
            # add to backup array

        if currentNodeX == 5:
            print("its five!")
            if np.logical_and(angles >= 45, angles <= 135).any():
                finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                checker3 = checker3 + 1

            else:
                 print("doodoo3")
                 backupAngleArray = np.append(backupAngleArray, angles)
                 backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                 backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])

        finalAngleIterator = finalAngleIterator + 1


    # If there are no nodes left after the 'behind' disqualification (and consequential angle disqualification), the program will choose a node that is over 45 degrees / behind the current node.
    if np.logical_and(checker2 != 0, checker3 == 0).any():
        print("BACKUP ARRAY !!!", len(backupAngleArray))
        totalFinalAngleArray = np.append(totalFinalAngleArray, backupAngleArray[0])
        finalxArray1 = np.append(finalxArray1, backupxArray[0])
        finalyArray1 = np.append(finalyArray1, backupyArray[0])

    if np.logical_and(checker2 == 0, checker3 == 0).any():
        print("move back")
        totalFinalAngleArray = np.append(totalFinalAngleArray, angleArray2[0])
        finalxArray1 = np.append(finalxArray1, minxArray4[0])
        finalyArray1 = np.append(finalyArray1, minyArray4[0])

    print("after angle", totalFinalAngleArray)


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

    # check to see if angle between start and end point and node is smaller than 25 degrees, if it is, add it to a final qualifying array. Choose the longest (last) node in array if there are multiple qualifying nodes.
    checker = False
    for num2 in totalFinalAngleArray:
        if np.logical_and(pastCurrentNodeX == finalxArray1[b], pastCurrentNodeY == finalyArray1[b]).any():
            print("past node1")
            backupFinalAngleArray = np.append(finalfinalArray, num2)
            backupFinalxArray = np.append(finalfinalxArray, finalxArray1[b])
            backupFinalyArray = np.append(finalfinalyArray, finalyArray1[b])
        else:
            print("normal chooser")
            finalfinalArray = np.append(finalfinalArray, num2)
            finalfinalxArray = np.append(finalfinalxArray, finalxArray1[b])
            finalfinalyArray = np.append(finalfinalyArray, finalyArray1[b])
            checker = True
        b = b + 1

    print(checker, len(finalAngleArray1))
    # If there are no nodes left over after all the disqualifiers, but the finalAngleArray1 (from 'behind' disqualifier) does not equal 0, the program will choose the first value of the array created after the 'behind' disqualification.
    p = 0
    if np.logical_and(checker == False, len(finalAngleArray1) > 0).any():
        for nums in finalAngleArray1:
            if np.logical_and(pastCurrentNodeX == finalxArray[p], pastCurrentNodeY == finalyArray[p]).any():
    #            backupFinalAngleArray = np.append(finalfinalArray, angleArray2[0])
    #            backupFinalxArray = np.append(finalfinalxArray, minxArray4[0])
    #            backupFinalyArray = np.append(finalfinalyArray, minyArray4[0])
    #        if np.logical_and(pastCurrentNodeX == finalxArray1[0], pastCurrentNodeY == finalyArray1[0]).any():
                print("past node2")
                backupFinalAngleArray = np.append(finalfinalArray, finalAngleArray1[p])
                backupFinalxArray = np.append(finalfinalxArray, finalxArray1[p])
                backupFinalyArray = np.append(finalfinalyArray, finalyArray1[p])
            else:
                print("alt chooser 1")
                finalfinalArray = np.append(finalfinalArray, finalAngleArray1[p])
                finalfinalxArray = np.append(finalfinalxArray, finalxArray[p])
                finalfinalyArray = np.append(finalfinalyArray, finalyArray[p])
                checker = True

#    # If there are no nodes left over after all the disqualifiers, and the finalAngleArray1 (from 'behind' disqualifier) equals 0, the program will choose the first value of the array created after the obstacle disqualification.
    f = 0
    if np.logical_and(checker == False, len(finalAngleArray1) == 0).any():
        for nums in angleArray2:
            if np.logical_and(pastCurrentNodeX == minxArray4[f], pastCurrentNodeY == minyArray4[f]).any():
                print("past node3")
                backupFinalAngleArray = np.append(finalfinalArray, angleArray2[f])
                backupFinalxArray = np.append(finalfinalxArray, minxArray4[f])
                backupFinalyArray = np.append(finalfinalyArray, minyArray4[f])
            else:
                print("alt chooser 2")
                finalfinalArray = np.append(finalfinalArray, angleArray2[f])
                finalfinalxArray = np.append(finalfinalxArray, minxArray4[f])
                finalfinalyArray = np.append(finalfinalyArray, minyArray4[f])
                checker = True
            f = f + 1

    if checker == False:
        finalfinalArray = np.append(finalfinalArray, backupFinalAngleArray[0])
        finalfinalxArray = np.append(finalfinalxArray, backupFinalxArray[0])
        finalfinalyArray = np.append(finalfinalyArray, backupFinalyArray[0])

    # Choosing of the next node. The program chooses the last value of the qualifying nodes, to ensure the longest path is being chosen.
    # The previous current node will be set to the past current node and added to the past current node array.
    print("chosen angle", finalfinalArray[-1])
    print("chosen x", finalfinalxArray[-1])
    print("chosen y", finalfinalyArray[-1])

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
    pastCurrentNodeXArray = np.append(pastCurrentNodeXArray, pastCurrentNodeX)
    pastCurrentNodeYArray = np.append(pastCurrentNodeYArray, pastCurrentNodeY)


    o = o + 1
    c = c + 1

# The new current node will have a green line drawn to it from the past current node.
q = 1
while q < o:
    point5 = [pastCurrentNodeXArray[q-1], pastCurrentNodeYArray[q-1]]
    point6 = [pastCurrentNodeXArray[q], pastCurrentNodeYArray[q]]
    x_values4 = [point5[0], point6[0]]
    y_values4 = [point5[1], point6[1]]
    ax.plot(x_values4, y_values4, '#76EE00')
    q = q + 1

# Calculates how long the program took to run from start to finish.
print("My program took", time.time() - start_time, "to run")

# Plotting nodes and the start and end nodes
plt.scatter(nodex, nodey, c = colors)
start = plt.plot(5,0,'go')
end = plt.plot(5,10,'ro')
plt.show()
