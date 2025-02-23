import API
import sys
import numpy as np
from dataclasses import dataclass
from collections import deque

#! ALWAYS START MOUSE FACING NORTH 
#0 = North
#1 = East
#2 = South
#3 = West
heading = 0

def blank_maze(dim):
    maze_ = np.ones((dim, dim))
    maze = np.Inf*maze_
    maze[(int) (dim/2 - 1)][(int) (dim/2 - 1)] = 0
    maze[(int) (dim/2)][(int) (dim/2 - 1)] = 0
    maze[(int) (dim/2 - 1)][(int) (dim/2)] = 0
    maze[(int) (dim/2)][(int) (dim/2)] = 0
    return maze

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():
    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")
    while True:
        if not API.wallFront(): log("Front: False") 
        else: log("Front: True")
        # if not API.wallBack(): log("Back: ") 
        # else: log("Back: True")
        if not API.wallRight(): log("Right: False") 
        else: log("Right: True")
        if not API.wallLeft(): log("Left: False") 
        else: log("Left: True")
        # if not API.wallFrontRight(): log("FrontRight: False") 
        # else: log("FrontRight: True")
        # if not API.wallFrontLeft(): log("FrontLeft: False") 
        # else: log("FrontLeft: True")
        # if not API.wallBackRight(): log("BackRight: False") 
        # else: log("BackRight: True")
        # if not API.wallBackLeft(): log("BackLeft: False") 
        # else: log("BackLeft: True")
        log("-----------------------------------------------------------")
        if not API.wallLeft():
            API.turnLeft()
        while API.wallFront():
            API.turnRight()
        API.moveForward()

def manhattan_nowall(maze):       
  #split maze into 4 quadrants and fill as if no walls are there
    dim = len(maze[0])
    val = 0
    row_inc = -1
    for i in range((int) (dim/2-1), -1, -1):
        row_inc+=1
        val = row_inc
        for j in range((int) (dim/2-1), -1, -1):
            maze[i][j] = val
            val = val+1
    val = 0
    row_inc = -1
    for i in range((int) (dim/2-1), -1, -1):
        row_inc+=1
        val = row_inc
        for j in range((int) (dim/2), dim):
            maze[i][j] = val
            val = val+1         
    val = 0
    row_inc = -1
    for i in range((int) (dim/2), dim):
        row_inc+=1
        val = row_inc
        for j in range((int) (dim/2-1), -1, -1):
            maze[i][j] = val
            val = val+1
    val = 0
    row_inc = -1
    for i in range((int)(dim/2), dim):
        row_inc+=1
        val = row_inc
        for j in range((int) (dim/2), dim):
            maze[i][j] = val
            val = val+1
    return maze

def set_borders(matrix, edges):
    for i in range(0, 4):
        if(i == 0 and edges[i]==1): # set first column of matrix to 1s
            matrix[:, 0] = 1
        if(i == 1 and edges[i]==1): #set first row to 1s
            matrix[0] = 1
        if(i == 2 and edges[i]==1): #set last column to 1s
            matrix[:,-1]=1
        if(i == 3 and edges[i]==1):
            matrix[-1] = 1
    return matrix

def transform_R_to_G(readings):
    global heading
    '''
    function takes in heading of mouse and wall readings relative to mouse.

    returns wall readings in global frame 
    '''
    
    # Get index of direction
    
    # Wall in global frame
    walls = readings #Readings = [L,R,F,B]

    if heading == 0: # north
        return walls
    
    elif (heading == 2): # south
        return [walls[1],walls[0],walls[3],walls[2]]
    
    elif (heading == 1): # east 
        return [walls[3],walls[2],walls[0],walls[1]]
    
    else: # west
        return [walls[2],walls[3],walls[1],walls[0]]
    
    
class Maze:
    '''
        maze class contains square matrix for manhattan distances,
        vertical wall rectangular matrix,
        and horizontal wall rectangular matrix
    '''
    def __init__(self, dim):
        self.md_matrix = blank_maze(dim)
        self.vw_matrix = set_borders(np.zeros((dim, dim+1)), [1, 0, 1, 0])
        self.hw_matrix = set_borders(np.zeros((dim+1, dim)), [0, 1, 0, 1])
        
    def update_walls(self, pos, wall_global):
        '''
            function takes in wall matrices, mouse position, and wall position around current cell
            
            returns updated wall matrices
        '''
        # pos = [row, col] (indices of manhattan distances grid)
        # walls: LRFB
        # top and left are same as position 
        self.hw_matrix[pos[0]][pos[1]] = wall_global[2]
        self.vw_matrix[pos[0]][pos[1]] = wall_global [0]
        
        #right and bottom are +1
        self.hw_matrix[pos[0]+1][pos[1]] = wall_global[3]
        self.vw_matrix[pos[0]][pos[1]+1] = wall_global[1]

    def floodfill(self,dim):

        # inputs: maze
        # ouptuts: updated manhattan distances

        # define center four target cells
        # (ROW,COL)
        top_left_target = (dim/2-1, dim/2-1)
        top_right_target = (dim/2-1, dim/2)
        bottom_left_target = (dim/2, dim/2-1)
        bottom_right_target = (dim/2, dim/2)

        #re-initialize
        self.md_matrix = blank_maze(dim)
    
        q = deque()
        q = deque([top_right_target,top_left_target,bottom_left_target,bottom_right_target])
        
        while(q): # while q still has values in it

            # Take the leading element, determine all accessible unwritten neighbors. Add 1 to manhattan distance and append.  
            # if cell != np.inf and if wall not blocking
                # +1 to it and add to queue
                
            coord = q.popleft() 
            # log(type(coord))
            # log(coord)

            row= int(coord[0])
            col= int(coord[1])   
          
            #check left & right neighbor
            #   horizontal wall 
            
            left_wall = self.hw_matrix[row][col]
            right_wall = self.hw_matrix[row][col+1]
            bottom_wall = self.vw_matrix[row+1][col]
            top_wall = self.vw_matrix[row][col]
            
            next_dist = self.md_matrix[row][col]+1
            #check if they are blank and accessible
            #check left neighbor first
            
            if(not left_wall and self.md_matrix[row][col-1]==np.inf):
                self.md_matrix[row][col-1] = next_dist
                q.append((row, col-1)) # append left neighbor because it is accessible and blank

            if(not right_wall and self.md_matrix[row][col+1]==np.inf):
                self.md_matrix[row][col+1] = next_dist
                q.append((row, col+1))

            if( not bottom_wall and self.md_matrix[row+1][col]==np.inf):
                self.md_matrix[row+1][col] = next_dist
                q.append((row+1, col)) 

            if(not top_wall and self.md_matrix[row-1][col]==np.inf):
                self.md_matrix[row-1][col] = next_dist
                q.append((row-1, col)) 

            
        return 
        
        
def check_cells(pos, dim, maze):
    '''
    check adjacent and accessible cells
    return array of tuples containing coordinates of accessible cells
    '''       
    
    '''
    bounds:
    left most cells: col = 0
    right most cells: col = dim-1
    top most cells: row = 0
    bottom most cells: row = dim-1
    ''' 
    accessible = []
    
    '''
    candidates array structure:
    each row is related to a direction. From rows 0-3--> left, top, right, bottom
    first two elements of each row is the coordinates of the cell in that direction
    next two elements of each row is index into vw or hw matrix that gives the wall in that direction
    last element is numerical identifier that tells what wall each row's data is related to
    '''
    
    candidates = [[pos[0], pos[1]-1, pos[0], pos[1], 3],      #left cell, left wall
                  [pos[0]-1, pos[1], pos[0], pos[1], 0],      #top cell, top wall
                  [pos[0], pos[1]+1, pos[0], pos[1]+1, 2],    #right cell, right wall
                  [pos[0]+1, pos[1], pos[0]+1, pos[1], 1]]    #bottom cell, bottom wall
    #check 4 cells around current cell
    
    for candidate in candidates:
        #check if within maze
        if(candidate[0]<0 or candidate[0]>=dim or candidate[1]<0 or candidate[1]>= dim):
            continue #not accessible because outside of maze
        else:
            wall_r = candidate[2]
            wall_c = candidate[3]
            #within the maze. so check walls
            if(candidate[4] == 3): #left wall
                if(maze.vw_matrix[wall_r][wall_c]):
                    continue                        #wall present, cant go in left direction
            elif(candidate[4] == 0): # top wall
                if(maze.hw_matrix[wall_r][wall_c]):
                    continue                        #wall present, cant go in top direction
            elif(candidate[4] == 2): # right wall
                if(maze.vw_matrix[wall_r][wall_c]):
                    continue                        #wall present, cant go in right direction
            else:                   # bottom wall
                if(maze.hw_matrix[wall_r][wall_c]):
                    continue
                
        #if there is no wall in direction of adjacent cell, then it is both ADJACENT and ACCESSIBLE
        accessible.append(candidate)
             
    return accessible
      
      
def move(dir, pos):
    global heading
    #dir 0(top cell), 1 (bottom cell), 2(right cell), 3 (left cell)
    #heading  0 (north), 1 (east ), 2 (south), 3 (west)
    pos_x = pos[1]
    pos_y = pos[0]
    if(dir == 3): # left
        pos_x = pos[1]-1
    elif(dir == 0): # top
        pos_y = pos[0]-1
    elif(dir == 2): # right
        pos_x = pos[1]+1
    else: # bottom
        pos_y = pos[0]+1
        
    #!turn left
    #facing forward (towards top) and want to go to cell to left OR
    # facing east and want to go to cell above
    # facing south and want to go to cell to right
    #facing west and want to go to cell below
    if((heading==0 and dir == 3) or (heading == 1 and dir == 0) or (heading == 2 and dir == 2) or (heading == 3 and dir == 1)):
        API.turnLeft()
        update_heading(-1)
    #!turn right
    #facing forward (towards top) and want to go to cell to right OR
    # facing east and want to go to cell below
    # facing south and want to go to cell to left
    #facing west and want to go to cell above
    elif((heading == 0 and dir == 2) or (heading == 1 and dir == 1) or (heading == 2 and dir == 0) or (heading == 3 and dir == 0)):
        API.turnRight()
        update_heading(1)
    #!u turn
    #facing forward (towards top) and want to go to cell to bottom OR
    # facing east and want to go to cell left
    # facing south and want to go to cell to top
    #facing west and want to go to cell right
    elif((heading == 0 and dir == 1) or (heading == 1 and dir == 3) or (heading == 2 and dir == 3) or (heading == 3 and dir == 2)):
        API.turnLeft()
        API.turnLeft()
        update_heading(2)
    
    #go forward after turn
    API.moveForward()
    return pos_x, pos_y   
        
def update_heading(turn):
    global heading
    heading = (heading + turn) % 4  # Right turn (+1), Left turn (-1), U-turn (+2 or -2), Straight (0) 
    
       
def traverse():
    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")
    
    #algorithm only works with symmetric dimensions
    dim = (int) (16)    
    
    #initialize maze
    maze = Maze(dim)
    #! after initializing maze with infinities, call floodfill to get manhattan distances into maze 
    #! assuming no walls
    
    maze.floodfill(dim)
    #row, column
    pos = [dim-1, 0] # set initial position to bottom left
    
    #heading North South East West. 
    destination_reached = (pos[0] == dim-1 and pos[1] == dim-1) or (pos[0] == dim and pos[1] == dim-1) or (pos[0] == dim and pos[1] == dim) or (pos[0] == dim-1 and pos[1] == dim)
    while(not destination_reached):
        #check walls (read IR in actual implementation)
        wallLeft = API.wallLeft()
        wallRight = API.wallRight()
        wallFront = API.wallFront()
        wallBack = False # cannot read behind b/c no IR
        
        #wall reading relative to mouse
        wall_mouse = [wallLeft, wallRight, wallFront, wallBack]
        
        #wall reading relative to global frame
        wall_global = transform_R_to_G(wall_mouse)
        
        #update horizontal and vertical walls
        maze.update_walls(pos, wall_global)
        
        #check adjacent and accessible cells
        accessible = check_cells(pos, dim, maze)
        num_accessible = len(accessible)
        move_flag = 0
        if(num_accessible == 0):
            maze.floodfill(dim)         #no accessible cells, recalculate floodfill
        else:
            curr_cell_md = maze.md_matrix[pos[0]][pos[1]]
            for cell in accessible:
                candidate_md = maze.md_matrix[cell[0]][cell[1]]
                if(candidate_md < curr_cell_md):
                    #move to cell whose manhattan distance is less than the current manhattan distance
                    pos_x, pos_y = move(cell[-1], heading, pos)
                    pos = [pos_y, pos_x] # update position to new position after moving
                    move_flag = 1 # indicate you have moved
                    break
            if(move_flag == 0): #you were not able to move because manhattan distances were not less
                maze.floodfill(dim)     #recalculate floodfill
        
        #should go to next iteration of loop whether floodfill calculated or moved 

    return #once mouse reaches center    
 
    

    

    
    
           
                
if __name__ == "__main__":
    traverse()
 