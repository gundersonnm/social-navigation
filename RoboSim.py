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


while i<100:

    # Assigns random x and y coordinate values for nodes
    nodex = round(random.randrange(0, dis_width - 30) / 10.0) * 10.0
    nodey = round(random.randrange(0, dis_width - 30) / 10.0) * 10.0

    # Draw individual rectangles for each node
    pygame.draw.rect(dis, white, [nodex, nodey, 15, 15])

    # Add each x, y , and distance (dis from start point to node) value for each node to individual array
    nodexArray.append(nodex)
    nodeyArray.append(nodey)
    nodeDistArray.append(math.sqrt((nodex - 800)**2 + (nodey - 1300)**2))

    # Create a zipped array of x, y, and distance values
    allValuesArray = zip(nodeDistArray, nodexArray, nodeyArray)
    allValuesArray = list(allValuesArray)

    # Sort array depending on distance value
    allValuesArray.sort()
    i = i + 1

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

# Define arbitrary variable values
r = 0
finalBehindIterator = 0
minxArrayint = 0
minyArrayint = 0
finalNodeArray = []

for nums in minDisArray:

    # assign min x and y to iterating values
    minxArrayint = minxArray[r]
    minyArrayint = minyArray[r]
    finalNodeArray.append(nums)

    # highlight min x y distance values values in blue
    pygame.draw.rect(dis, blue, [minxArrayint, minyArrayint, 15, 15])
    pygame.draw.line(dis, blue, (815, 1315), (minxArrayint, minyArrayint), 3)

#    if obstacle in way
#        disqualify

    # disqualify and remove values that are behind start point from final node array
    if minyArrayint >= 1300:
        del finalNodeArray[finalBehindIterator]
        finalBehindIterator = finalBehindIterator + 1

    # find value for angle between direct start to end line and each node point
    angleX = abs(815 - minxArrayint)
    angleY = abs(1315 - minyArrayint)
    angle = math.degrees(math.atan(angleX / angleY))

    r = r + 1

print(finalNodeArray)


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
