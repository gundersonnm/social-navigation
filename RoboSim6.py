import math
import random

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

white = (255, 255, 255)
yellow = (255, 255, 102)
black = (0, 0, 0)
red = (213, 50, 80)
green = (0, 255, 0)
blue = (50, 153, 213)
import pygame
import time
import random
import math

pygame.init()

# Assigns RGB color values
white = (255, 255, 255)
yellow = (255, 255, 102)
black = (0, 0, 0)
red = (213, 50, 80)
green = (0, 255, 0)
blue = (50, 153, 213)
gray = (131, 139, 139)

# Assigns size of simulation screen
dis_width = 1600
dis_height = 1400
dis = pygame.display.set_mode((dis_width, dis_height))

# Sets name of simulation screen
pygame.display.set_caption('RoboSim Visualizer')

# Define arbitrary iterators and empty arrays
i = 0
x = 0
nodeDistArray = []
nodexArray = []
nodeyArray = []
minDisArray = []
minxArray = []
minyArray = []
currentNodeX = 815
currentNodeY = 1315
nodexArray1 = []
nodeyArray1 = []
o = 0
pastCurrentNodeX = 0
pastCurrentNodeY = 0

# creation of nodes

while i<100:
        # Assigns random x and y coordinate values for nodes
        nodex = round(random.randrange(0, dis_width - 30) / 10.0) * 10.0
        nodey = round(random.randrange(0, dis_height - 30) / 10.0) * 10.0

        # Draw individual rectangles for each node
        pygame.draw.rect(dis, white, [nodex, nodey, 15, 15])

        # Add each x, y , and distance (dis from start point to node) value for each node to individual array
        nodexArray.append(nodex)
        nodeyArray.append(nodey)

        i = i + 1

nodexArray.append(815)
nodeyArray.append(115)
check1 = False

while o < 20:

    if currentNodeX == 815.0 and currentNodeY == 115.0:
        check1 = True

    if check1 == True:
        break

    #print(nodexArray)
    #print(len(nodexArray))
    #print(nodeyArray)

    nodexArray1 = []
    nodeyArray1 = []
    nodeDistArray = []

    p = 0
    x = 0
    for jj in nodexArray:

        if nodexArray[x] != currentNodeX and nodeyArray[x] != currentNodeY:

            nodexArray1.append(nodexArray[x])
            nodeyArray1.append(nodeyArray[x])
            newx = nodexArray1[p]
            newy = nodeyArray1[p]
            nodeDistArray.append(math.sqrt((newx - currentNodeX)**2 + (newy - currentNodeY)**2))

            # Create a zipped array of x, y, and distance values
            allValuesArray = zip(nodeDistArray, nodexArray1, nodeyArray1)
            allValuesArray = list(allValuesArray)

            # Sort array depending on distance value
            allValuesArray.sort()

    #    if nodexArray[p] == currentNodeX and nodeyArray[p] == currentNodeY:
    #        p = p - 1

            p = p + 1

        x = x + 1

    #print(nodexArray1)
    #print(len(nodexArray1))
    #print(nodeyArray1)

    minxArray = []
    minyArray = []
    minDisArray = []

    # adding distances to minDisArray
    for dist in allValuesArray:
        minDisArray.append(dist[0])

    # adding x coordinate values to minxArray
    for numx in allValuesArray:
        minxArray.append(numx[1])

    # adding y coordinate values to minyArray
    for numy in allValuesArray:
        minyArray.append(numy[2])

    # altering min arrays to only include the first 10 index values
    del minDisArray[10:]
    del minxArray[10:]
    del minyArray[10:]

    l = 0
    checkcheck = False
    for kk in minxArray:
        if kk == 815 and minyArray[l] == 115:
            currentNodeX = kk
            currentNodeY = minyArray[l]
            pygame.draw.line(dis, green, (currentNodeX, currentNodeY), (xcoord, ycoord), 3)
            checkcheck = True
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
        pygame.draw.rect(dis, blue, [minxArrayint, minyArrayint, 15, 15])
        pygame.draw.line(dis, blue, (currentNodeX, currentNodeY), (minxArrayint, minyArrayint), 3)


        # find value for angle between direct start to end line and each node point, save to array
        angleX = (minxArrayint - currentNodeX)
        angleY = (minyArrayint - currentNodeY)
        angle = math.degrees(math.atan(angleY / angleX))
        angleReal = 90 - abs(angle)
        angleArray.append(angleReal)

        r = r + 1

    # if obstacle in way
        # disqualify
    print(finalNodeArray)
    # define arbitrary variables for 'behind' disqualification
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
        if nums1 < currentNodeY:
                finalAngleArray1.append(angleArray[finalBehindIterator])
                finalxArray.append(minxArray[finalBehindIterator])
                finalyArray.append(minyArray[finalBehindIterator])
#                checker2 = checker2 + 1
        finalBehindIterator = finalBehindIterator + 1
        continue

#    if checker2 == 0:
#        finalAngleArray1.append(angleArray[0])
#        finalxArray.append(minxArray[0])
#        finalyArray.append(minyArray[0])

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
            finalxArray1.append(minxArray3)
            finalyArray1.append(minyArray3)
            totalFinalAngleArray.append(angles)
            checker3 = checker3 + 1
        finalAngleIterator = finalAngleIterator + 1

    if checker3 == 0 or checker2 == 0:
        if pastCurrentNodeX == minxArray[0] and pastCurrentNodeY == minyArray[0]:
            totalFinalAngleArray.append(angleArray[1])
            finalxArray1.append(minxArray[1])
            finalyArray1.append(minyArray[1])
        else:
            totalFinalAngleArray.append(angleArray[0])
            finalxArray1.append(minxArray[0])
            finalyArray1.append(minyArray[0])

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
            finalfinalArray.append(num2)
            finalfinalxArray.append(finalxArray1[b])
            finalfinalyArray.append(finalyArray1[b])
            checker = True
        b = b + 1

    if checker == False:
        finalfinalArray.append(totalFinalAngleArray[0])
        finalfinalxArray.append(finalxArray1[0])
        finalfinalyArray.append(finalyArray1[0])

    # choose the last element in the x and y coordinates to draw a green line
    # line goes from start point to next
    xcoord = finalfinalxArray[-1]
    ycoord = finalfinalyArray[-1]
    pastCurrentNodeX = currentNodeX
    pastCurrentNodeY = currentNodeY
    pygame.draw.line(dis, green, (currentNodeX, currentNodeY), (xcoord, ycoord), 3)
    currentNodeX = xcoord
    currentNodeY = ycoord

    o = o + 1


clock = pygame.time.Clock()

def gameLoop():
    game_over = False
    game_close = False

    while not game_over:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                game_over = True


        while game_close == True:
            dis.fill(blue)

            pygame.display.update()

        #STARTING POINT
        pygame.draw.rect(dis, green, [800, 1300, 30, 30])

        #END POINT
        pygame.draw.rect(dis, red, [800, 100, 30, 30])

        # STRAIGHT LINE
        pygame.draw.line(dis, gray, (815, 1315), (815, 115), 3)


        pygame.display.update()

    pygame.quit()
    quit()


gameLoop()

gray = (131, 139, 139)

N = 100

nodex = np.random.rand(N)
nodey = np.random.rand(N)
colors = (0,0,0)

# Define arbitrary iterators and empty arrays
i = 0
x = 0
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

# creation of nodes

#while i<100:
        # Assigns random x and y coordinate values for nodes
        #nodex = round(random.randrange(0, dis_width - 30) / 10.0) * 10.0
        #nodey = round(random.randrange(0, dis_height - 30) / 10.0) * 10.0

        # Draw individual rectangles for each node
        #pygame.draw.rect(dis, white, [nodex, nodey, 15, 15])

        # Add each x, y , and distance (dis from start point to node) value for each node to individual array
nodexArray.append(nodex)
nodeyArray.append(nodey)

        #i = i + 1

nodexArray.append(0.5)
nodeyArray.append(1)
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

            nodexArray1.append(nodexArray[x])
            nodeyArray1.append(nodeyArray[x])
            newx = nodexArray1[p]
            newy = nodeyArray1[p]
            nodeDistArray.append(math.sqrt((newx[p] - currentNodeX)**2 + (newy[p] - currentNodeY)**2))

            # Create a zipped array of x, y, and distance values
            allValuesArray = zip(nodeDistArray, nodexArray1, nodeyArray1)
            allValuesArray = list(allValuesArray)

            # Sort array depending on distance value
            allValuesArray.sort()

    #    if nodexArray[p] == currentNodeX and nodeyArray[p] == currentNodeY:
    #        p = p - 1

            p = p + 1

        x = x + 1

    minxArray = []
    minyArray = []
    minDisArray = []

    # adding distances to minDisArray
    for dist in allValuesArray:
        minDisArray.append(dist[0])

    # adding x coordinate values to minxArray
    for numx in allValuesArray:
        minxArray.append(numx[1])

    # adding y coordinate values to minyArray
    for numy in allValuesArray:
        minyArray.append(numy[2])

    # altering min arrays to only include the first 10 index values
    del minDisArray[10:]
    del minxArray[10:]
    del minyArray[10:]

    l = 0
    checkcheck = False
    for kk in minxArray:
        if np.logical_and(kk == 0.5, minyArray[l] == 1.0).any():
            currentNodeX = kk
            currentNodeY = minyArray[l]
            x_values = [currentNodeX, xcoord]
            y_values = [currentNodeY, ycoord]
            plt.plot(x_values, y_values)
            checkcheck = True
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
        plt.plot(minxArrayint, minyArrayint, 'bo')
        np.append(minxArrayint, currentNodeX)
        np.append(minyArrayint, currentNodeY)
        x_values1 = [minxArrayint]
        y_values1 = [minxArrayint]
        plt.plot(x_values1, y_values1)

        # find value for angle between direct start to end line and each node point, save to array
        angleX = (minxArrayint - currentNodeX)
        angleY = (minyArrayint - currentNodeY)
        angle = math.degrees(math.atan(angleY[r] / angleX[r]))
        angleReal = 90 - abs(angle)
        angleArray.append(angleReal)

        r = r + 1


    # if obstacle in way
        # disqualify

    # define arbitrary variables for 'behind' disqualification
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
        if (nums1 < currentNodeY).any():
                finalAngleArray1.append(angleArray[finalBehindIterator])
                finalxArray.append(minxArray[finalBehindIterator])
                finalyArray.append(minyArray[finalBehindIterator])
#                checker2 = checker2 + 1
        finalBehindIterator = finalBehindIterator + 1
        continue

#    if checker2 == 0:
#        finalAngleArray1.append(angleArray[0])
#        finalxArray.append(minxArray[0])
#        finalyArray.append(minyArray[0])

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
            finalxArray1.append(minxArray3)
            finalyArray1.append(minyArray3)
            totalFinalAngleArray.append(angles)
            checker3 = checker3 + 1
        finalAngleIterator = finalAngleIterator + 1

    if checker3 == 0 or checker2 == 0:
        if np.logical_and(pastCurrentNodeX == minxArray[0], pastCurrentNodeY == minyArray[0]).any():
            totalFinalAngleArray.append(angleArray[1])
            finalxArray1.append(minxArray[1])
            finalyArray1.append(minyArray[1])
        else:
            totalFinalAngleArray.append(angleArray[0])
            finalxArray1.append(minxArray[0])
            finalyArray1.append(minyArray[0])

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
            finalfinalArray.append(num2)
            finalfinalxArray.append(finalxArray1[b])
            finalfinalyArray.append(finalyArray1[b])
            checker = True
        b = b + 1

    if checker == False:
        finalfinalArray.append(totalFinalAngleArray[0])
        finalfinalxArray.append(finalxArray1[0])
        finalfinalyArray.append(finalyArray1[0])

    # choose the last element in the x and y coordinates to draw a green line
    # line goes from start point to next
    xcoord = finalfinalxArray[-1]
    ycoord = finalfinalyArray[-1]
    pastCurrentNodeX = currentNodeX
    pastCurrentNodeY = currentNodeY
    np.append(xcoord, currentNodeX)
    np.append(ycoord, currentNodeY)
    x_values2 = [xcoord]
    y_values2 = [ycoord]
    plt.plot(x_values2, y_values2)
    currentNodeX = xcoord
    currentNodeY = ycoord

    o = o + 1

plt.figure('RoboSim Visualizer')
plt.scatter(x,y, c = colors)
start = plt.plot(0.5,0,'go')
end = plt.plot(0.5,1,'ro')
plt.show()
