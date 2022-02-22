#!/usr/bin/env python
import rospy

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
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():



        start_time = time.time()


        # Assigns RGB color values to be used in simulation
        white = (255, 255, 255)
        yellow = (255, 255, 102)
        black = (0, 0, 0)
        poop = (158, 47, 35)
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
        xArray = []
        yArray = []
        xminArray = []
        yminArray = []
        xmaxArray = []
        ymaxArray = []
        x_obs1 = []
        y_obs1 = []
        # Creation of obstacles, and adding their coordinate values to xmin, ymin, xmax, and ymax Arrays. We use these values to calculate intersection points.
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')

        #patch = plt.Circle((0, 0), 0.4, fc='r')

        #def init():
        #    patch.center = (5, 5)
        #    ax.add_patch(patch)
        #    return patch,
        #
        #def animate(i):
        #    x, y = patch.center
        #    x = i
        #    y = i
        #    patch.center = (x, y)
        #    return patch,

        #anim = animation.FuncAnimation(fig, animate,
         #  init_func=init,
         #  frames=40,
         #  interval=200,
         #  blit=True)

        plt.xlim([0, 11])
        plt.ylim([0, 10.5])
        n=20
        for i in range(0,n):
            x_obs = random.uniform(0, 9)
            y_obs = random.uniform(0, 9)
            ax.add_patch(matplotlib.patches.Rectangle((x_obs-0.1, y_obs-0.1),1,1, facecolor='#A23E2A'))
            ax.add_patch(matplotlib.patches.Rectangle((x_obs, y_obs),0.8,0.8, facecolor='#D1634D'))
            xmin = x_obs- 0.1
            ymin = y_obs - 0.1
            xmax = x_obs + 1
            ymax = y_obs + 1
            xArray = np.append(xArray, x_obs)
            yArray = np.append(yArray, y_obs)
            x_obs1 = np.append(x_obs1, x_obs)
            y_obs1 = np.append(y_obs1, y_obs)
            xminArray = np.append(xminArray, xmin)
            yminArray = np.append(yminArray, ymin)
            xmaxArray = np.append(xmaxArray, xmax)
            ymaxArray = np.append(ymaxArray, ymax)

        print(x_obs1, 'OBSTACLESSSSS')
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
        dynamicxArray = []
        dynamicyArray = []
        dynamicx1Array = []
        dynamicy1Array = []
        dynamicx2Array = []
        dynamicy2Array = []



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

            # Dynamic Obstacle Values
            dynamicX = o + 3
            dynamicY = o + 1
            Dynamicxmin = dynamicX - 0.2
            Dynamicymin = dynamicY - 0.2
            Dynamicxmax = dynamicX + 1.2
            Dynamicymax = dynamicY + 1.2


            dynamicxArray = np.append(dynamicxArray, dynamicX)
            dynamicyArray = np.append(dynamicyArray, dynamicY)

            # Dynamic Obstacle Values
            dynamicX1 = o
            dynamicY1 = 10 - o
            Dynamicxmin1 = dynamicX1 - 0.2
            Dynamicymin1 = dynamicY1 - 0.2
            Dynamicxmax1 = dynamicX1 + 1.2
            Dynamicymax1 = dynamicY1 + 1.2

            dynamicx1Array = np.append(dynamicx1Array, dynamicX1)
            dynamicy1Array = np.append(dynamicy1Array, dynamicY1)

            # Dynamic Obstacle Values
            dynamicX2 = o + 4
            dynamicY2 = o
            Dynamicxmin2 = dynamicX2 - 0.2
            Dynamicymin2 = dynamicY2 - 0.2
            Dynamicxmax2 = dynamicX2 + 1.2
            Dynamicymax2 = dynamicY2 + 1.2

            dynamicx2Array = np.append(dynamicx2Array, dynamicX2)
            dynamicy2Array = np.append(dynamicy2Array, dynamicY2)


            #if o == 1:
            #    xminArray = xminArray[:-1]
            #    yminArray = yminArray[:-1]
            #    xmaxArray = xmaxArray[:-1]
            #    ymaxArray = ymaxArray[:-1]

            if o>=1:
                xminArray = xminArray[:-2]
                yminArray = yminArray[:-2]
                xmaxArray = xmaxArray[:-2]
                ymaxArray = ymaxArray[:-2]

            xArray = np.append(xArray, dynamicX)
            yArray = np.append(yArray, dynamicY)
            xminArray = np.append(xminArray, Dynamicxmin)
            yminArray = np.append(yminArray, Dynamicymin)
            xmaxArray = np.append(xmaxArray, Dynamicxmax)
            ymaxArray = np.append(ymaxArray, Dynamicymax)

            #if o == 1:
            #    xminArray = xminArray[:-1]
            #    yminArray = yminArray[:-1]
            #    xmaxArray = xmaxArray[:-1]
            #    ymaxArray = ymaxArray[:-1]

            xArray = np.append(xArray, dynamicX1)
            yArray = np.append(yArray, dynamicY1)
            xminArray = np.append(xminArray, Dynamicxmin1)
            yminArray = np.append(yminArray, Dynamicymin1)
            xmaxArray = np.append(xmaxArray, Dynamicxmax1)
            ymaxArray = np.append(ymaxArray, Dynamicymax1)

            print(dynamicX, dynamicY, "DYNAMIC X Y COORDINATES")
            print(dynamicX1, dynamicY1, "DYNAMIC1 X1 Y1 COORDINATES1")
            print(xminArray, yminArray, "XMINARRAY YMINARRAY")

            # This checks if the path planner has reached the final node, and if it has, breaks the all encapsuling loop.
            if np.logical_and(currentNodeX == finalNodex, currentNodeY == finalNodey).any():
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
                if np.logical_and(kk == finalNodex, minyArray[l] == finalNodey).any():
                    point1 = [currentNodeX, currentNodeY]
                    point2 = [finalNodex, finalNodey]
                    x_values = [point1[0], point2[0]]
                    y_values = [point1[1], point2[1]]
                    ax.plot(x_values, y_values, '#76EE00')
                    checkcheck = True
                    pastCurrentNodeXArray = np.append(pastCurrentNodeXArray, x_values)
                    pastCurrentNodeXArray = np.append(pastCurrentNodeXArray, 5)
                    pastCurrentNodeYArray = np.append(pastCurrentNodeYArray, y_values)
                    pastCurrentNodeYArray = np.append(pastCurrentNodeYArray, 10)
                    print(pastCurrentNodeXArray, 'at the end lkakjsfdpoiiwej')
                    print(pastCurrentNodeYArray, 'at the end lkakjsfdpoiiwej')
                    o = o + 1
                    print(o, 'before whattttt')
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
                while b < 22:
                    nodeDistance = 0
                    #distance calculation from node to edge of obstacle
                    nodeDistance = math.sqrt((xminArray[b] - currentNodeX)**2 + (yminArray[b] - currentNodeY)**2)
                    if nodeDistance <= 4:
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
            bestfinalxArray1 = []
            bestfinalyArray1 = []
            besttotalFinalAngleArray = []
            backtrackIterator = 0
            superCloseArray = []
            superCloseXArray = []
            superCloseYArray = []

            # Angle disqualifier! This will check to see if angle between start to finish line to node is larger than 45 degrees, and do not add the node to the new array if it is.
            for angles in finalAngleArray1:

                # if we're not choosing any past nodes
                if np.logical_or(finalyArray[backtrackIterator] != pastCurrentNodeY, finalxArray[backtrackIterator] != pastCurrentNodeX).any():

                    # if we have moved more than 2 nodes forward
                    if len(pastCurrentNodeXArray)>=2:

                        # if we arent choosing a past node x2
                        if np.logical_or(finalyArray[backtrackIterator] != pastCurrentNodeYArray[-2], finalxArray[backtrackIterator] != pastCurrentNodeXArray[-2]).any():

                            # if we're to the left of the final node, trying to move right
                            if currentNodeX < finalNodex:
                                if np.logical_and(angles < 90, angles > 0).any():
                                    #minxArray3 = finalxArray[finalAngleIterator]
                                    #minyArray3 = finalyArray[finalAngleIterator]
                                    finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                                    finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                                    totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                                    checker3 = checker3 + 1


                                if np.logical_and(angles >= 90, angles <= 180).any():
                                    backupAngleArray = np.append(backupAngleArray, angles)
                                    backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                                    backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
                                    # add to backup array

                                if angles == 0:
                                    #minxArray3 = finalxArray[finalAngleIterator]
                                    #minyArray3 = finalyArray[finalAngleIterator]
                                    superCloseXArray = np.append(superCloseXArray, finalxArray[finalAngleIterator])
                                    superCloseYArray = np.append(superCloseYArray, finalyArray[finalAngleIterator])
                                    totalFinalAngleArray = np.append(superCloseArray, finalAngleArray1[finalAngleIterator])
                                    checker3 = checker3 + 1
                                    print("T H I S H A P P E N E D   MOVE RIGHT")


                            # if we're to the right of the final node
                            if currentNodeX > finalNodex:
                                if np.logical_and(angles > 90, angles < 180).any():
                                    #minxArray3 = finalxArray[finalAngleIterator]
                                    #minyArray3 = finalyArray[finalAngleIterator]
                                    finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                                    finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                                    totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                                    checker3 = checker3 + 1

                                if np.logical_or(angles <= 90, angles >= 0):
                                    backupAngleArray = np.append(backupAngleArray, angles)
                                    backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                                    backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
                                # add to backup array

                                if angles == 180:
                                    superCloseXArray = np.append(superCloseXArray, finalxArray[finalAngleIterator])
                                    superCloseYArray = np.append(superCloseYArray, finalyArray[finalAngleIterator])
                                    totalFinalAngleArray = np.append(superCloseArray, finalAngleArray1[finalAngleIterator])
                                    checker3 = checker3 + 1
                                    print("T H I S H A P P E N E D   MOVE LEFT")

                            # if we're straight on with the node
                            if angles == 90:
                                bestfinalxArray1 = np.append(bestfinalxArray1, finalxArray[finalAngleIterator])
                                bestfinalyArray1 = np.append(bestfinalyArray1, finalyArray[finalAngleIterator])
                                besttotalFinalAngleArray = np.append(besttotalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                                checker3 = checker3 + 1

                            else:
                                 backupAngleArray = np.append(backupAngleArray, angles)
                                 backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                                 backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])

                    # if we are choosing past nodes
                    else:

                        if currentNodeX < finalNodex:
                            if np.logical_and(angles <= 90, angles >= 0).any():
                                #minxArray3 = finalxArray[finalAngleIterator]
                                #minyArray3 = finalyArray[finalAngleIterator]
                                finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                                finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                                totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                                checker3 = checker3 + 1


                            if np.logical_and(angles >= 90, angles <= 180).any():
                                backupAngleArray = np.append(backupAngleArray, angles)
                                backupxArray = np.append(backupxArray, finalxArray[finalAngleIterator])
                                backupyArray = np.append(backupyArray, finalyArray[finalAngleIterator])
                                # add to backup array

                        if currentNodeX > finalNodex:
                            if np.logical_and(angles >= 90, angles <= 180).any():
                                #minxArray3 = finalxArray[finalAngleIterator]
                                #minyArray3 = finalyArray[finalAngleIterator]
                                finalxArray1 = np.append(finalxArray1, finalxArray[finalAngleIterator])
                                finalyArray1 = np.append(finalyArray1, finalyArray[finalAngleIterator])
                                totalFinalAngleArray = np.append(totalFinalAngleArray, finalAngleArray1[finalAngleIterator])
                                checker3 = checker3 + 1

                            if np.logical_or(angles <= 90, angles >= 0):
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

            print("besttotalFinalAngleArray", besttotalFinalAngleArray)
            print("totalFinalAngleArray", totalFinalAngleArray)
            print("backupAngleArray", backupAngleArray)
            print("backupBackupAngleArray", backupBackupAngleArray)

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
            if currentNodeY <= 5:
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
            if currentNodeY > 5:
                if len(finalxArray1) != 0:
                    choosingxArray = np.append(choosingxArray, finalxArray1[-1])
                    choosingyArray = np.append(choosingyArray, finalyArray1[-1])
                    choosingAngleArray = np.append(choosingAngleArray, totalFinalAngleArray[-1])
                    print("drifting")
                    checker1 = True
                if checker1 == False:
                    if len(bestfinalxArray1) != 0:
                        choosingxArray = np.append(choosingxArray, bestfinalxArray1[-1])
                        choosingyArray = np.append(choosingyArray, bestfinalyArray1[-1])
                        choosingAngleArray = np.append(choosingAngleArray, besttotalFinalAngleArray[-1])
                        print("move straight")
                        checker1 = True
            if currentNodeY == finalNodey:
                if len(superCloseArray) != 0:
                    if checker1 == False:
                        choosingxArray = np.append(choosingxArray, superCloseXArray[-1])
                        choosingyArray = np.append(choosingyArray, superCloseYArray[-1])
                        choosingAngleArray = np.append(choosingAngleArray, superCloseArray[-1])
                        print("move horizontal")
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
        end = plt.plot(finalNodex,finalNodey,'ro')
        plt.show()



        fig, ax = plt.subplots()

        rx = []
        ry = []
        obstaclex = []
        obstacley = []
        obstaclex1 = []
        obstacley1 = []
        obstaclex2 = []
        obstacley2 = []

        dynamicxArray = np.append(dynamicxArray, np.linspace(17, 17+2, o+1))
        dynamicyArray = np.append(dynamicyArray, np.linspace(17, 17+2, o+1))
        dynamicx1Array = np.append(dynamicx1Array, np.linspace(17, 17+2, o+1))
        dynamicy1Array = np.append(dynamicy1Array, np.linspace(17, 17+2, o+1))
        dynamicx2Array = np.append(dynamicx2Array, np.linspace(17, 17+2, o+1))
        dynamicy2Array = np.append(dynamicy2Array, np.linspace(17, 17+2, o+1))

        patch = patches.Rectangle((0, 0), 0, 0, fc='y')

        def init():
            ax.add_patch(patch)
            return patch,

        # function that draws each frame of the animation
        def animate(mom):
            rx.append(pastCurrentNodeXArray[mom])
            ry.append(pastCurrentNodeYArray[mom])
            obstaclex.append(dynamicxArray[mom])
            obstacley.append(dynamicyArray[mom])
            obstaclex1.append(dynamicx1Array[mom])
            obstacley1.append(dynamicy1Array[mom])
            obstaclex2.append(dynamicx2Array[mom])
            obstacley2.append(dynamicy2Array[mom])
            patch.set_width(1)
            patch.set_height(1)
            patch.set_xy([obstaclex[mom], obstacley[mom]])
            print(mom)
            ax.clear()
            ax.plot(rx, ry)
            ax.plot(rx[-1], ry[-1])
        #    ax.plot(obstaclex, obstacley)
        #    ax.plot(obstaclex1, obstacley1)
            ax.scatter(nodex, nodey, c = colors)
            ax.plot(5,0,'go')
            ax.plot(finalNodex,finalNodey,'ro')
            n=20
            for i in range(0,n):
                x = x_obs1[i]
                y = y_obs1[i]
                ax.add_patch(matplotlib.patches.Rectangle((x-0.1, y-0.1),1,1, facecolor='#A23E2A'))
                ax.add_patch(matplotlib.patches.Rectangle((x, y),0.8,0.8, facecolor='#D1634D'))
            n=20
            for i in range(0,n):
                x = obstaclex[mom]
                y = obstacley[mom]
                x1 = obstaclex1[mom]
                y1 = obstacley1[mom]
                x2 = obstaclex2[mom]
                y2 = obstacley2[mom]
                ax.add_patch(matplotlib.patches.Rectangle((x-0.1, y-0.1),1,1, facecolor='#35AC43'))
                ax.add_patch(matplotlib.patches.Rectangle((x, y),0.8,0.8, facecolor='#76EE00'))
                ax.add_patch(matplotlib.patches.Rectangle((x1-0.1, y1-0.1),1,1, facecolor='#35AC43'))
                ax.add_patch(matplotlib.patches.Rectangle((x1, y1),0.8,0.8, facecolor='#76EE00'))
                ax.add_patch(matplotlib.patches.Rectangle((x2-0.1, y2-0.1),1,1, facecolor='#35AC43'))
                ax.add_patch(matplotlib.patches.Rectangle((x2, y2),0.8,0.8, facecolor='#76EE00'))

            ax.set_xlim([0, 11])
            ax.set_ylim([0, 10.5])

            return patch,

            # run the animation
        ani = FuncAnimation(fig, animate, init_func = init, frames=o+1, interval=500, repeat=False)
        #ani.save('myAnimation.gif', writer='imagemagick', fps=1)

        plt.show()
        
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
