import API
import sys
import numpy as np
from dataclasses import dataclass

#not currently using 
# def check_boundaries(x, y, dim):
#     #top, bottom, left, right
#     valid = [1, 1, 1, 1]
#     if(x == 0): #if mouse at top edge
#         valid[0] = 0
#         if(y == 0): # if mouse at top left corner
#             valid[2] = 0
#         elif(y == dim-1):#if mouse at top right corner
#             valid[3] = 0
#     elif(x == dim-1):
#          valid[1] = 0
#          if(y == 0): # if mouse at bot left corner
#             valid[2] = 0
#          elif(y == dim-1):#if mouse at bot right corner
#             valid[3] = 0
#     elif(y == 0): valid[2] = 0
#     elif(y == dim-1): valid[3] = 0
    
#     return valid

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

def transform_R_to_G(heading, readings):

    '''
    function takes in heading of mouse and wall readings relative to mouse.

    returns wall readings in global frame 
    '''
    
    # Get index of direction
    direction = np.argmax(heading) 
    
    # Wall in global frame
    walls = readings #Readings = [L,R,F,B]

    if direction == 0: # north
        return walls
    
    elif (direction == 1): # south
        return [walls[1],walls[0],walls[3],walls[2]]
    
    elif (direction == 2): # east 
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
        #pos = [row, col] (indices of manhattan distances grid)
        # walls: LRFB
        #top and left are same as position 
        self.hw_matrix[pos[0]][pos[1]] = wall_global[2]
        self.vw_matrix[pos[0]][pos[1]] = wall_global [0]
        
        #right and bottom are +1
        self.hw_matrix[pos[0]+1][pos[1]] = wall_global[3]
        self.vw_matrix[pos[0]][pos[1]+1] = wall_global[1]
            
def traverse():
    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")
    
    #algorithm only works with symmetric dimensions
    dim = (int) (16)    
    
    #initialize maze
    maze = Maze(dim)
    
    #row, column
    pos = [dim-1, 0] # set initial position to bottom left
    
    #heading North South East West. 
    #! ALWAYS START MOUSE FACING NORTH 
    heading = [1, 0, 0, 0]
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
        
            
                
if __name__ == "__main__":
    main()
 