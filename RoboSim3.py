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

currentNodeX = 815
currentNodeY = 1315

# Define arbitrary iterators and empty arrays
i = 0
nodexArray = []
nodeyArray = []
nodex = 0
nodey = 0
m = 0

while i<100:

    # Assigns random x and y coordinate values for nodes
    nodex = round(random.randrange(0, dis_width - 30) / 10.0) * 10.0
    nodey = round(random.randrange(0, dis_height - 30) / 10.0) * 10.0

    # Draw individual rectangles for each node
    pygame.draw.rect(dis, white, [nodex, nodey, 15, 15])

    # Add each x, y value for each node to individual array
    nodexArray.append(nodex)
    nodexArray.append(800)
    nodeyArray.append(nodey)
    nodeyArray.append(100)

    i = i + 1

while m < 5:
#while currentNodeX != 800.0 and currentNodeY != 100.0:
    nodeDistArray = []
    minDisArray = []
    minxArray = []
    minyArray = []

    k = 0
    while k < 100:
        dist = math.sqrt((nodexArray[k] - currentNodeX)**2 + (nodeyArray[k] - currentNodeY)**2)
        nodeDistArray.append(dist)

        # Create a zipped array of x, y, and distance values
        allValuesArray = zip(nodeDistArray, nodexArray, nodeyArray)
        allValuesArray = list(allValuesArray)

        # Sort array depending on distance value
        allValuesArray.sort()

        k = k + 1

#    if currentNodeX != 800.0 and currentNodeY != 100.0:
#        break

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

    d = 0
    breakChecker = False
    for num3 in minxArray:
        if num3 == 800 and minyArray[d] == 100:
            pygame.draw.line(dis, green, (currentNodeX, currentNodeY), (num3, minyArray[d]), 3)
            breakChecker = True
        d = d + 1

    if breakChecker == True:
        break

    # Define arbitrary variable values
    r = 0
    minxArrayint = 0
    minyArrayint = 0
    finalNodeArray = []
    angleArray = []

    for nums in minDisArray:

        minxArrayint = minxArray[r]
        minyArrayint = minyArray[r]

        # assign min x and y to iterating values
        if minxArrayint != currentNodeX and minyArrayint != currentNodeY:
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

    # define arbitrary variables for 'behind' disqualification
    finalBehindIterator = 0
    j = 0
    finalAngleArray1 = []
    finalxArray = []
    finalyArray = []
    minyArray2 = 0
    minxArray2 = 0
    angleIterator = 0

    print(angleArray)
    print(minxArray)
    print(minyArray)

    # check to see if node is behind start point, disqualify node if it is.
    for nums1 in minyArray:
        if nums1 < currentNodeY:
            finalAngleArray1.append(angleArray[finalBehindIterator])
            finalxArray.append(minxArray[finalBehindIterator])
            finalyArray.append(minyArray[finalBehindIterator])
            finalBehindIterator = finalBehindIterator + 1

    print(finalAngleArray1)
    print(finalxArray)
    print(finalyArray)

    # define arbitrary variables for angle disqualification
    finalAngleIterator = 0
    totalFinalAngleArray = []
    minxArray3 = []
    minyArray3 = []
    finalxArray1= []
    finalyArray1 = []

    # check to see if angle between start to finish line to node is larger than 45 degrees
    # disqualify node if it is.
    for angles in finalAngleArray1:
        if angles < 45:
            minxArray3 = finalxArray[finalAngleIterator]
            minyArray3 = finalyArray[finalAngleIterator]
            finalxArray1.append(minxArray3)
            finalyArray1.append(minyArray3)
            totalFinalAngleArray.append(angles)
        finalAngleIterator = finalAngleIterator + 1

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
            finalfinalArray.append(num2)
            finalfinalxArray.append(finalxArray1[b])
            finalfinalyArray.append(finalyArray1[b])
            checker = True
        b = b + 1

    if checker == False:
        badxcoord = finalfinalxArray[-1]
        badycoord = finalfinalyArray[-1]
        pygame.draw.line(dis, green, (currentNodeX, currentNodeY), (badxcoord, badycoord), 3)


    # choose the last element in the x and y coordinates to draw a green line
    # line goes from start point to next best node
    xcoord = finalfinalxArray[-1]
    ycoord = finalfinalyArray[-1]
    if checker == True:
        pygame.draw.line(dis, green, (currentNodeX, currentNodeY), (xcoord, ycoord), 3)

    currentNodeX = xcoord
    currentNodeY = ycoord
    print(finalfinalArray)
    m = m + 1

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
