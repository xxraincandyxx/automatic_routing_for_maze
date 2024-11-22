# Library
import os
import cv2
import math
import heapq
import numpy as np
import pandas as pd


# Main Class
class direction():
    # Initialize the class
    def __init__(self, rows: int, cols: int) -> None:
        self.toggle = False
        
        # Initialize maze size
        self.rows = rows
        self.cols = cols
        
        # Create an unknown maze
        self.maze = np.full((rows, cols), -1, dtype=np.int8)
        
        # Variables definition
        self.vision = []
        self.path = []
        
        # Path buffer heap
        self.path_tmp = []
        heapq.heapify(self.path_tmp)
    
    # Modify the path
    def path_mod(self, path: list[tuple]) -> None:
        self.path.append(path)
        
    # Modify the heap
    def path_heap(self, loc: tuple) -> tuple:
        if self.path_tmp is None:
            return None
        
        _, path = heapq.heappop(self.path_tmp)
        return (path[0] - loc[0], path[1] - loc[1])
    
    # Clear path_tmp cache
    def path_clr(self) -> tuple:
        self.path_tmp = []
        return None
    
    # Define the heuristic function
    def heuristic(self, a: tuple, b: tuple) -> float:
        return math.dist(a, b)  # Return the distance

    # Black block search
    def block_search(self, start: tuple, goal: tuple) -> list[tuple]:
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Clear path_tmp cache
                self.path_clr()
                
                idx = 0
                while current in came_from:
                    heapq.heappush(self.path_tmp, (idx, current))
                    current = came_from[current]
                    idx -= 1
                    
                # Trigger the toggle
                self.toggle = False
                
                return self.path_heap(start)
            
            for direction in [
                    (-1,  1), ( 0,  1), ( 1,  1),
                    (-1,  0),           ( 1,  0),
                    (-1, -1), ( 0, -1), ( 1, -1)
                ]:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                
                if 0 <= neighbor[0] < self.rows and 0 <= neighbor[1] < self.cols and self.maze[neighbor[0]][neighbor[1]] != 1:
                    tentative_g_score = g_score[current] + 1
                    
                    if self.maze[neighbor[0], neighbor[1]] == -1:
                        # Clear path_tmp cache
                        self.path_clr()
                        
                        idx = 0
                        while current in came_from:
                            heapq.heappush(self.path_tmp, (idx, current))
                            current = came_from[current]
                            idx -= 1
                            
                        # Trigger the toggle
                        self.toggle = False
                        
                        return self.path_heap(start)
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        
        return None  # No path found

    # Find the nearest black block
    def nearest_bblock_path(self, loc: tuple) -> list[tuple]:
        block_set = []
        
        for i in range(self.rows):
            for j in range(self.cols):
                if self.maze[i, j] == -1:
                    heapq.heappush(block_set, (math.dist((i, j), loc), (i, j)))
                if self.maze[i, j] == 2:
                    direct = self.destination(start=loc, goal=(i, j))
                    if direct is not None:
                        return direct
                    
        while block_set:
            _, block_loc = heapq.heappop(block_set)
            
            direct = self.block_search(start=loc, goal=block_loc)
            
            if direct is not None:
                block_set = []
                return direct
        
        return None # Path not found

    # Check the destination
    def destination(self, start: tuple, goal: tuple) -> tuple:
        open_set = []
        heapq.heappush(open_set, (0, start))
        direction = (0, 0)
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Clear path_tmp cache
                self.path_clr()
                
                idx = 0
                while current in came_from:
                    heapq.heappush(self.path_tmp, (idx, current))
                    current = came_from[current]
                    idx -= 1
                    
                # Trigger the toggle
                self.toggle = True
                
                return self.path_heap(start)
            
            for direction in [
                    (-1,  1), ( 0,  1), ( 1,  1),
                    (-1,  0),           ( 1,  0),
                    (-1, -1), ( 0, -1), ( 1, -1)
                ]:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                
                if 0 <= neighbor[0] < self.rows and \
                    0 <= neighbor[1] < self.cols and \
                    (self.maze[neighbor[0]][neighbor[1]] == 0 or \
                    self.maze[neighbor[0]][neighbor[1]] == 2):
                    tentative_g_score = g_score[current] + 1
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # No path found
    
    # Visualize the nearby blocks
    def visualize(self, x_buffer: np.array, y_buffer: np.array, v_buffer: np.array, length: int) -> tuple:
        dest = None
        
        # Vision info load
        for i in range(length):
            # Read vision to the maze
            x = x_buffer[i]
            y = y_buffer[i]
            self.maze[x, y] = v_buffer[i]
            
            # Check destination
            if v_buffer[i] == 2:
                dest = (x, y)
            
            # Initialize vision
            self.vision.append((x, y))
            
        return dest
    
    # Return the intended direcion
    def direct(self, loc: tuple) -> tuple:
        # Variables init
        current = loc
        maxBlock = 0
        move_dir = (0, 0)
        
        # Direction loop
        for direction in [
                    (-1,  1), ( 0,  1), ( 1,  1),
                    (-1,  0),           ( 1,  0),
                    (-1, -1), ( 0, -1), ( 1, -1)
                ]:
            
            # Calculate the number of the black block to be discovered
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            if not (0 <= neighbor[0] < self.rows and 0 <= neighbor[1] <self.cols) or self.maze[neighbor[0], neighbor[1]] != 0: continue
            
            block = 0
            for coordinate in self.vision:
                    if 0 <= coordinate[0] + direction[0] < self.rows and \
                       0 <= coordinate[1] + direction[1] <self.cols and \
                       self.maze[coordinate[0] + direction[0], coordinate[1] + direction[1]] == -1:
                        block += 1
            
            if block > maxBlock:
                maxBlock = block
                move_dir = direction
        
        return move_dir
    
    def final_check(self, start: tuple) -> tuple:
        for i in range(self.rows):
            for j in range(self.cols):
                if self.maze[i, j] == 2:
                    direct = self.destination(start=start, goal=(i, j))
                    return direct
        
        return None  # Goal not found

dir = None

# Initialize the map
def init(width: int, height: int) -> None:
    global dir
    dir = direction(cols=width, rows=height)
    return None

# Result
def result(x: int, y: int, x_buffer: np.array, y_buffer: np.array, v_buffer: np.array, length: int) -> tuple:
    global dir
    
    assert dir is not None, 'ERROR: Class not initialized!'
    
    direct = (0, 0)
    
    # Add current location to path
    dir.path_mod(path=(x, y))
    
    # Vision init
    dest = dir.visualize(x_buffer=x_buffer, y_buffer=y_buffer, v_buffer=v_buffer, length=length)
    
    # Destination rush
    if dir.toggle == True:
        # print((x +direct[0], y +direct[1]))
        direct = dir.path_heap(loc=(x, y))
        assert direct is not None, 'ERROR: \'direct\' is \'NoneType\'!'
        return direct
    
    # Destination check
    if isinstance(dest, tuple):
        # print(dest)
        direct = dir.destination(start=(x, y), goal=dest)
        if direct is not None:
            return direct
        
    # Find the direction
    direct = dir.nearest_bblock_path(loc=(x, y))
    if direct is not None:
        return direct
    
    direct = dir.final_check(start=(x, y))
    assert direct is not None, 'ERROR: \'direct\' is \'NoneType\'!'
    return direct
        

maze = np.array([
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1],
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1],
    [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1],
    [0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1],
    [0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1],
    [0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1],
    [0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
    [1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1],
    [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
    [0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1],
    [0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1],
    [0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
    [1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
    [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 2]], dtype=np.int8)

inverted_array = np.array([[not element for element in row] for row in maze])
image_array = (inverted_array * 255).astype(np.uint8)
image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
cv2.imwrite("map.png", image)

rows = 22
cols = 32
init(width=32, height=22)

def plot_res(maze=maze, path=None, filename='tmp.png') -> None:
    
    res = maze.copy()
    row = rows
    col = cols
    
    rgb = np.full((rows, cols, 3), fill_value=0, dtype=np.int8)
    
    for i in range(row):
        for  j in range(col):
            if res[i, j] == 1:
                rgb[i, j] = np.array([0, 0, 0])
                continue
            
            if res[i, j] == 0:
                rgb[i, j] = np.array([255, 255, 255])
                continue
            
            if res[i, j] == -1:
                rgb[i ,j] = np.array([0, 0, 0])
 
    if path:
        for step in path:
            rgb[step[0], step[1]] += np.array([50, 50, 0])
                
    image = np.array(rgb).astype(np.uint8)
    # image = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
    cv2.imwrite(filename, image)
    
start = (0, 0)
goal = (21, 31)
current = start
radius = 2
path = []
print(np.size(maze))

while current != goal:
    path.append(current)
    
    x_range = (max(0, current[0]-radius), min(rows, current[0]+radius+1))
    y_range = (max(0, current[1]-radius), min(cols, current[1]+radius+1))
    
    x_buffer = []
    y_buffer = []
    v_buffer = []
    
    length = 0
    for i in range(x_range[0], x_range[1]):        
        for j in range(y_range[0], y_range[1]):
            x_buffer.append(i)
            y_buffer.append(j)
            v_buffer.append(maze[i, j])
            length += 1
    
    direct = result(x=current[0], y=current[1], x_buffer=x_buffer, y_buffer=y_buffer, v_buffer=v_buffer, length=length)
    
    plot_res(maze=dir.maze, path=dir.path)
    
    current = (current[0] + direct[0], current[1] + direct[1])
    
print(path)