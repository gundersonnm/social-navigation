Ni = initial state
Nf = goal state
n = number of nodes on map

Loop (until Nc = Nf)

  Nc = Current state

  # Take in locations of high cost areas from cost map algorithm, store values
  CostMap() = occupancy probability, high cost locations

  loop (n times)   # Plotting predefined pattern of nodes over size of the cost map
    Plot nodes over size(CostMap)

    if (n is next to Nc)     # store the location value of the nodes surrounding Nc
      Nclose = n


  loop (Nclose)   # Choose lowest cost node path from 8 options

    # if there is a predicted collision in node path, increase cost based on occupancy probability
    If (Nclose collide with high cost area)
      Cost ++

    # if the node path would move horizontal or backwards, increase cost value of that node path
    # Horizontal node paths have lower cost than moving backwards
    If (Nclose angle > 180)
      Cost ++

    If (Nclose = Nc)     # The planner has the option to remain stationary to allow obstacles to pass
      Cost ++

    If (moving straight forward)     # Promote the moving directly forward by decreasing cost of that path
      Cost --

    # Promote the Nclose path drifting towards the position of the goal state by decreasing cost of that path
    # Lower cost to drift than move straight, this difference exponentiates as Nc gets closer to Nf
    If (Nclose x position ~= Nf x position)
      Cost --

    # A new "current node (Nc)" is chosen based on which path has the lowest cost
    Nc = lowest cost Nclose
