import numpy as np
import math
import heapq
import random 
target_error = 0.1

UNMAPPED_CELL = 1
OPEN_CELL = 2
WALL_CELL = 3
FRONTIER_CELL = 4

# def costmap(data,width,height): #convert data from 1D array to 2D array 
#     data = np.array(data).reshape(height,width)
#     condition = np.logical_and(data > -1, data < 50)
#     data = np.where(condition, 0, data) #make everything < 5e+01 but not -1 open space
#     data = np.where(data > 50, 100, data) #make everything > 5e+01 but not -1 obstacle
#     wall = np.where(data == 100)
#     for i in range(-expansion_size,expansion_size+1):
#         for j in range(-expansion_size,expansion_size+1):
#             if i  == 0 and j == 0:
#                 continue
#             x = wall[0]+i
#             y = wall[1]+j
#             x = np.clip(x,0,height-1) #make sure cells changing is within the map
#             y = np.clip(y,0,width-1)
#             data[x,y] = 100
    
#     return data

def find_frontier_cells(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == OPEN_CELL: #open space 
                #convert cell into frontier cell as long as an unmapped cell -1 is adjacent to it 
                if i > 0 and matrix[i-1][j] == 1: #top
                    matrix[i][j] = FRONTIER_CELL
                elif i < len(matrix)-1 and matrix[i+1][j] == 1: #bottom
                    matrix[i][j] = FRONTIER_CELL
                elif j > 0 and matrix[i][j-1] == 1: #left 
                    matrix[i][j] = FRONTIER_CELL
                elif j < len(matrix[i])-1 and matrix[i][j+1] == 1: #right 
                    matrix[i][j] = FRONTIER_CELL
    return matrix

def assign_groups(matrix):
    group_num = 1 #key in dictionary
    groups = {}
    for i in range(len(matrix)): #shd work with len -1 i think
        for j in range(len(matrix[0])):
            #do dfs to find neighbouring frontier cells 
            if matrix[i][j] == FRONTIER_CELL:
                group_num = dfs(matrix, i, j, group_num, groups)
    return groups

def dfs(matrix, i, j, group_num, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group_num
    if matrix[i][j] != FRONTIER_CELL:
        return group_num
    
    # if frontier cell found
    # if prev cell was also frontier append to same grp
    if group_num in groups:
        groups[group_num].append((i, j))
    
    # else start new group 
    else:
        groups[group_num] = [(i, j)]

    matrix[i][j] = 0 
    #change frontier cell back to open space cell so both functions dont search same area again
    dfs(matrix, i + 1, j, group_num, groups)
    dfs(matrix, i - 1, j, group_num, groups)
    dfs(matrix, i, j + 1, group_num, groups)
    dfs(matrix, i, j - 1, group_num, groups)
    dfs(matrix, i + 1, j + 1, group_num, groups) # bottom right 
    dfs(matrix, i - 1, j - 1, group_num, groups) # top left 
    dfs(matrix, i - 1, j + 1, group_num, groups) # top right 
    dfs(matrix, i + 1, j - 1, group_num, groups) # bottom left 

    #increase group number when all frontier cells in 1 grp found 
    return group_num + 1

#############################################################
def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False

def findClosestGroup(matrix,groups, current,resolution,originX,originY):
    print("FINDCLOSESTGROUP")
    targetP = None
    distances = []
    paths = []
    score = []
    max_score = -1 #max score index
    middles = []
    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],[p[1] for p in groups[i][1]]) 
        path = astar(matrix, current, middle)
        path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
        # path = [(p[1],p[0]) for p in path]
        total_distance = pathLength(path)
        distances.append(total_distance)
        paths.append(path)

    for i in range(len(distances)):
        if distances[i] == 0:
            score.append(0)
        else:
            score.append(len(groups[i][1])/distances[i])
    for i in range(len(distances)):
        if distances[i] > target_error*3:
            if max_score == -1 or score[i] > score[max_score]:
                max_score = i
    
    if max_score != -1:
        targetP = paths[max_score]
    # else: #If the groups are closer than target_error*2 distance, it selects a random point as the target. This allows the robot to escape some situations
    #     index = random.randint(0,len(groups)-1)
    #     target = groups[index][1]
    #     target = target[random.randint(0,len(target)-1)]
    #     path = astar(matrix, current, target)
    #     targetP = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
    return targetP

def fGroups(groups):
    print("FGROUPS")
    print(groups)
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 0]    
    return top_five_groups

def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid


def costmap(data,width,height,resolution, expansion_size):
    # data = np.array(data).reshape(height,width)
    wall = np.where(data == 1)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data
################################################# TURKISH GUY CODE ABOVE

# def calculate_centroid(group):
#     x_coords = []
#     y_coords = []
#     for cell in group:
#         x_coords.append(cell[0])
#         y_coords.append(cell[1])
#     n = len(x_coords)
#     sum_x = sum(x_coords)
#     sum_y = sum(y_coords)
#     mean_x = sum_x / n
#     mean_y = sum_y / n
#     centroid = (int(mean_x), int(mean_y))
#     return centroid 


def find_neighbors(node, occupancy_data, nrows, ncols):
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    for dir in directions:
        neighbor = (node[0] + dir[0], node[1] + dir[1])
        if 0 <= neighbor[0] < nrows and 0 <= neighbor[1] < ncols:
            if occupancy_data[neighbor[0], neighbor[1]] != 3:  # Check if it is not wall
                neighbors.append(neighbor)
    return neighbors

def a_star(start, target, occupancy_data, nrows, ncols):
    open_set = []
    closed_set = set()
    heapq.heappush(open_set, (0, start))
    came_from = {}

    g_score = {start: 0}
    f_score = {start: heuristic(start, target)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == target:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        closed_set.add(current)

        for neighbor in find_neighbors(current, occupancy_data, nrows, ncols):
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[current] + 1  # Assuming cost of moving from one cell to another is 1

            if neighbor not in [x[1] for x in open_set] or tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, target)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

def findClosesttoGoal(cmap, groups, target_x, target_y, curr_x, curr_y, resolution):
    print("FINDCLOSESTTOGOAL")
    centroids = []
    dists = []
    for group in groups:
        centroid = calculate_centroid(group)
        if ableToTravel(cmap, centroid[0], centroid[1], curr_x, curr_y, resolution):
            centroids.append()
    for centroid in centroids:
        dists.append(math.dist(centroid, (target_x, target_y)))
    min_index = dists.index(min(dists))
    if centroids:
        print("No Centroid Found")
        return None
    return centroids[min_index][0], centroids[min_index][1]

def ableToTravel(map_odata, curr_x_grid, curr_y_grid, target_x_grid, target_y_grid, resolution):
    margin_grid = (int)(0.1/resolution)
    #print(margin_grid)
    if (target_y_grid == curr_y_grid):
        for x in range (curr_x_grid, target_x_grid, 1):
            y = target_y_grid
            if np.any(map_odata[math.floor(y)-margin_grid:math.ceil(y)+margin_grid+1, x] == 100):
                return False    # meet a wall
    elif (target_x_grid == curr_x_grid):
        for y in range (curr_y_grid, target_y_grid, 1):
            x = target_x_grid
            if np.any(map_odata[math.floor(y)-margin_grid:math.ceil(y)+margin_grid+1, x] == 100):
                return False    # meet a wall
    else:
        gradient = float(target_y_grid - curr_y_grid)/float(target_x_grid - curr_x_grid) 
        x_init = curr_x_grid #for iteration
        y_init = curr_y_grid
        x_dest = target_x_grid
        if curr_x_grid > target_x_grid:
            x_init = target_x_grid
            y_init = target_y_grid
            x_dest = curr_x_grid
        for x in range (x_init + 1, x_dest, 1):
            y = gradient * (x - x_init) + y_init
            if map_odata[math.ceil(y), x] == 100 or map_odata[math.floor(y), x] == 100:
                return False    # meet a wall
    return True




    


