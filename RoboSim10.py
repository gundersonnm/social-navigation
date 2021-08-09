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
N = 100

nodex = np.random.rand(N)
nodey = np.random.rand(N)
colors = (0,0,0)


# ROBOSIM7!!!!

nodeDistArray = []
node1xArray = []
node1yArray = []
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

node1xArray = np.append(nodex, currentNodeX)
node1yArray = np.append(nodey, currentNodeY)

node1xArray = np.append(node1xArray, 0.5)
node1yArray = np.append(node1yArray, 1)

print('starting robosim7')
print(node1xArray)

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
    for jj in node1xArray:

        if np.logical_and(node1xArray[x] != currentNodeX, node1yArray[x] != currentNodeY).any():

            nodexArray1 = np.append(nodexArray1, node1xArray[x])
            nodeyArray1 = np.append(nodeyArray1, node1yArray[x])
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


# ROBOSIM9


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
nodeDistArray9 = []
nodexArray9 = []
nodeyArray9 = []
minDisArray9 = []
minxArray9 = []
minyArray9 = []
currentNodeX9 = 0.5
currentNodeY9 = 0
nodexArray19 = []
nodeyArray19 = []
o9 = 0
pastCurrentNodeX9 = 0
pastCurrentNodeY9 = 0

# Adding the current node and final node to x and y arrays
nodexArray9 = np.append(nodex, currentNodeX9)
nodeyArray9 = np.append(nodey, currentNodeY9)

nodexArray9 = np.append(nodexArray9, 0.5)
nodeyArray9 = np.append(nodeyArray9, 1)

print(nodexArray9)

check19 = False

# Looping through multiple iterations of nodes to reach the final node
while o9 < 20:

    # Checking if its reached the final node
    if np.logical_and(currentNodeX9 == 0.5, currentNodeY9 == 1.0).any():
        check19 = True

    if check19 == True:
        break

    # Building arrays filled with distances from current nodes to nodes aroundd it
    nodexArray19 = []
    nodeyArray19 = []
    nodeDistArray9 = []
    p9 = 0
    x9 = 0
    for jj9 in nodexArray9:

        if np.logical_and(nodexArray9[x9] != currentNodeX9, nodeyArray9[x9] != currentNodeY9).any():

            nodexArray19 = np.append(nodexArray19, nodexArray9[x9])
            nodeyArray19 = np.append(nodeyArray19, nodeyArray9[x9])
            nodeDistArray9.append(math.sqrt((nodexArray19[p9] - currentNodeX9)**2 + (nodeyArray19[p9] - currentNodeY9)**2))

            allValuesArray9 = zip(nodeDistArray9, nodexArray19, nodeyArray19)
            allValuesArray9 = list(allValuesArray9)

            allValuesArray9.sort()

            p9 = p9 + 1
        x9 = x9 + 1

    # Building arrays with the closest 10 nodes to current node
    minxArray9 = []
    minyArray9 = []
    minDisArray9 = []

    for dist9 in allValuesArray9:
        minDisArray9 = np.append(minDisArray9, dist9[0])

    for numx9 in allValuesArray9:
        minxArray9 = np.append(minxArray9, numx9[1])

    for numy9 in allValuesArray9:
        minyArray9 = np.append(minyArray9, numy9[2])

    minDisArray9 = minDisArray9[:10]
    minxArray9 = minxArray9[:10]
    minyArray9 = minyArray9[:10]

    l9 = 0
    checkcheck9 = False
    for kk9 in minxArray9:
        if np.logical_and(kk9 == 0.5, minyArray9[l9] == 1.0).any():
            point19 = [currentNodeX9, currentNodeY9]
            point29 = [0.5, 1.0]
            x_values9 = [point19[0], point29[0]]
            y_values9 = [point19[1], point29[1]]
            ax.plot(x_values9, y_values9, '#76EE00')
            checkcheck9 = True
            break
        l9 = l9 + 1

    if checkcheck9 == True:
        break

    # Define arbitrary variable values
    r9 = 0
    minxArrayint9 = 0
    minyArrayint9 = 0
    finalNodeArray9 = []
    angleArray9 = []

    for nums9 in minDisArray9:

        # assign min x and y to iterating values
        minxArrayint9 = minxArray9[r9]
        minyArrayint9 = minyArray9[r9]
        finalNodeArray9.append(nums9)

        # highlight min x y distance values values in blue
        ax.plot(minxArrayint9, minyArrayint9, 'bo')
        np.append(minxArrayint9, currentNodeX9)
        np.append(minyArrayint9, currentNodeY9)

        X,Y = np.meshgrid(minxArrayint9, minyArrayint9)
        positions9 = np.vstack([Y.ravel(), X.ravel()])
        origin9 = (currentNodeX9, currentNodeY9)
        for i9 in range(len(positions9)):
            for j9 in range(len(positions9[i9])):
                x19 = positions9[1][j9]
                y19 = positions9[0][j9]
                line9 = LineString([origin9, (x19, y19)])
                x29, y29 = line9.xy
                ax.plot(0, 0, x29, y29, color = 'blue')

        # find value for angle between direct start to end line and each node point, save to array
        angleX9 = (minxArrayint9 - currentNodeX9)
        angleY9 = (minyArrayint9 - currentNodeY9)
        angle9 = math.degrees(math.atan(angleY9 / angleX9))
        angleReal9 = 90 - abs(angle9)
        angleArray9 = np.append(angleArray9, angleReal9)


        r9 = r9 + 1

    print(minxArray9)
    print(minyArray9)

    # Obstacle disqualifier!
    # Checking intersections between nodes and obstacles
    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    # Given three colinear points p, q, r, the function checks if
    # point q lies on line segment 'pr'
    def onSegment(p, q, r):
        if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
               (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
            return True
        return False

    def orientation(p, q, r):
        # to find the orientation of an ordered triplet (p,q,r)
        # function returns the following values:
        # 0 : Colinear points
        # 1 : Clockwise points
        # 2 : Counterclockwise

        val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
        if (val > 0):

            # Clockwise orientation
            return 1
        elif (val < 0):

            # Counterclockwise orientation
            return 2
        else:

            # Colinear orientation
            return 0

    # The main function that returns true if
    # the line segment 'p1q1' and 'p2q2' intersect.
    def doIntersect(p1,q1,p2,q2):

        # Find the 4 orientations required for
        # the general and special cases
        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        # General case
        if ((o1 != o2) and (o3 != o4)):
            return True

        # Special Cases

        # p1 , q1 and p2 are colinear and p2 lies on segment p1q1
        if ((o1 == 0) and onSegment(p1, p2, q1)):
            return True

        # p1 , q1 and q2 are colinear and q2 lies on segment p1q1
        if ((o2 == 0) and onSegment(p1, q2, q1)):
            return True

        # p2 , q2 and p1 are colinear and p1 lies on segment p2q2
        if ((o3 == 0) and onSegment(p2, p1, q2)):
            return True

        # p2 , q2 and q1 are colinear and q1 lies on segment p2q2
        if ((o4 == 0) and onSegment(p2, q1, q2)):
            return True

        # If none of the cases
        return False

    # Testing each side of the obstacles against the lines from the current node to each possible node
    for angles9 in minxArray9:

        # Define arbitrary variables and arrays with checkers
        h9 = 0
        minxArray49 = []
        minyArray49 = []
        angleArray29 = []
        for angless9 in minxArray9:
            obstacleChecker9 = 0
            # Driver program to test above functions:
            p1 = Point(currentNodeX9, currentNodeY9)
            q1 = Point(minxArray9[h9], minyArray9[h9])
            p2 = Point(xminArray[h9], yminArray[h9])
            q2 = Point(xminArray[h9], ymaxArray[h9])

            if doIntersect(p1, q1, p2, q2):
                print("Yes")
                obstacleChecker9 = obstacleChecker9 + 1
            else:
                print("No")

            p1 = Point(currentNodeX9, currentNodeY9)
            q1 = Point(minxArray9[h9], minyArray9[h9])
            p2 = Point(xminArray[h9], ymaxArray[h9])
            q2 = Point(xmaxArray[h9], ymaxArray[h9])

            if doIntersect(p1, q1, p2, q2):
                print("Yes")
                obstacleChecker9 = obstacleChecker9 + 1

            else:
                print("No")

            p1 = Point(currentNodeX9, currentNodeY9)
            q1 = Point(minxArray9[h9], minyArray9[h9])
            p2 = Point(xmaxArray[h9], ymaxArray[h9])
            q2 = Point(xmaxArray[h9], yminArray[h9])

            if doIntersect(p1, q1, p2, q2):
                print("Yes")
                obstacleChecker9 = obstacleChecker9 + 1

            else:
                print("No")

            p1 = Point(currentNodeX9, currentNodeY9)
            q1 = Point(minxArray9[h9], minyArray9[h9])
            p2 = Point(xmaxArray[h9], yminArray[h9])
            q2 = Point(xminArray[h9], yminArray[h9])

            if doIntersect(p1, q1, p2, q2):
                print("Yes")
                obstacleChecker9 = obstacleChecker9 + 1

            else:
                print("No")

            if obstacleChecker9 == 0:
                minxArray49 = np.append(minxArray49, minxArray9[h9])
                minyArray49 = np.append(minyArray49, minyArray9[h9])
                angleArray29 = np.append(angleArray29, angleArray9[h9])

            print(obstacleChecker9)

            h9 = h9 + 1

    print(minxArray9)
    print(minyArray9)

    # Behind disqualifier!
    # Define arbitrary variables for 'behind' disqualification
    finalBehindIterator9 = 0
    j9 = 0
    finalAngleArray19 = []
    finalxArray9 = []
    finalyArray9 = []
    minyArray29 = 0
    minxArray29 = 0
    angleIterator9 = 0
    checker29 = 0

    # check to see if node is behind start point, disqualify node if it is.
    for nums19 in minyArray49:
        if nums19 > currentNodeY9:
            finalxArray9 = np.append(finalxArray9, minxArray49[finalBehindIterator9])
            finalyArray9 = np.append(finalyArray9, minyArray49[finalBehindIterator9])
            finalAngleArray19 = np.append(finalAngleArray19, angleArray29[finalBehindIterator9])
            checker29 = checker29 + 1
        finalBehindIterator9 = finalBehindIterator9 + 1
        continue

    # define arbitrary variables for angle disqualification
    finalAngleIterator9 = 0
    totalFinalAngleArray9 = []
    minxArray39 = []
    minyArray39 = []
    finalxArray19 = []
    finalyArray19 = []
    checker39 = 0

    # Angle disqualifier!
    # check to see if angle between start to finish line to node is larger than 45 degrees
    # disqualify node if it is.
    for angles9 in finalAngleArray19:
        if angles9 < 45:
            minxArray39 = finalxArray9[finalAngleIterator9]
            minyArray39 = finalyArray9[finalAngleIterator9]
            finalxArray19 = np.append(finalxArray19, minxArray39)
            finalyArray19 = np.append(finalyArray19, minyArray39)
            totalFinalAngleArray9 = np.append(totalFinalAngleArray9, angles9)
            checker39 = checker39 + 1
        finalAngleIterator9 = finalAngleIterator9 + 1

    if checker39 == 0 or checker29 == 0:
        if pastCurrentNodeX9 == minxArray9[0] and pastCurrentNodeY9 == minyArray9[0]:
            totalFinalAngleArray9 = np.append(totalFinalAngleArray9, angleArray9[1])
            finalxArray19 = np.append(finalxArray19, minxArray9[1])
            finalyArray19 = np.append(finalyArray19, minyArray9[1])
        else:
            totalFinalAngleArray9 = np.append(totalFinalAngleArray9, angleArray9[0])
            finalxArray19 = np.append(finalxArray9, minxArray9[0])
            finalyArray19 = np.append(finalyArray9, minyArray9[0])

    # define arbitrary variables for final chosen node desicion making
    finalDistanceIterator9 = 0
    b9 = 0
    minxArrayint19 = 0
    minyArrayint19 = 0
    finalfinalxArray9 = []
    finalfinalyArray9 = []
    finalfinalArray9 = []

    # check to see if angle between start and end point and node is smaller than 25 degrees
    # if it is, add it to a final qualifying array
    # choose the longest (last) node in array
    checker9 = False
    for num29 in totalFinalAngleArray9:
        if num29 <= 25:
            finalfinalArray9 = np.append(finalfinalArray9, num29)
            finalfinalxArray9 = np.append(finalfinalxArray9, finalxArray19[b9])
            finalfinalyArray9 = np.append(finalfinalyArray9, finalyArray19[b9])
            checker9 = True
        b9 = b9 + 1

    if checker9 == False:
        finalfinalArray9 = np.append(finalfinalArray9, totalFinalAngleArray9[0])
        finalfinalxArray9 = np.append(finalfinalxArray9, finalxArray19[0])
        finalfinalyArray9 = np.append(finalfinalyArray9, finalyArray19[0])

    # choose the last element in the x and y coordinates to draw a green line
    # line goes from start point to next
    xcoord9 = finalfinalxArray9[-1]
    ycoord9 = finalfinalyArray9[-1]
    pastCurrentNodeX9 = currentNodeX9
    pastCurrentNodeY9 = currentNodeY9
    point39 = [pastCurrentNodeX9, pastCurrentNodeY9]
    point49 = [xcoord9, ycoord9]
    x_values39 = [point39[0], point49[0]]
    y_values39 = [point39[1], point49[1]]
    ax.plot(x_values39, y_values39, '#76EE00')
    currentNodeX9 = xcoord9
    currentNodeY9 = ycoord9

    o9 = o9 + 1

# Plotting nodes and the start and end nodes
plot1 = plt.figure(1)
plt.scatter(nodex, nodey, c = colors)
start = plt.plot(0.5,0,'go')
end = plt.plot(0.5,1,'ro')
plt.show()
