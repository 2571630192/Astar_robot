"""
A* robot path planning algorithm
Author: Kaushik Balakrishnan, PhD
Email: kaushikb258@gmail.com
"""

import numpy as np

# set the grid of the world
# 0: free space; 1: obstacle
grid = [[0, 0, 1, 0, 0, 0, 1, 0, 0, 0], [0, 0, 1, 0, 0, 0, 1, 0, 0, 0], [0, 0, 1, 0, 1, 0, 1, 0, 1, 0], [0, 0, 1, 0, 1, 0, 0, 0, 1, 0], [0, 0, 0, 0, 1, 0, 0, 0, 1, 0]]
grid = np.array(grid).astype(int)

nrows, ncols = grid.shape
print "---------------------------------------------------"
print "nrows= ", nrows
print "ncols= ", ncols
print "grid= "
print grid
print "---------------------------------------------------"

# set coordinates of start and end locations
start = [0, 0]
end = [nrows-1, ncols-1]


# set path plan
path = np.zeros([nrows,ncols]).astype(int)
path[start[0],start[1]] = 1


# Manhattan distance 
def manhattan(i,j):
 return (np.abs(i-end[0]) + np.abs(j-end[1]))


# Compute Heuristic wrt end location
H = np.zeros([nrows,ncols]).astype(int)
H = np.array(H).astype(int)
for i in range(nrows):
 for j in range(ncols):
   H[i,j] = manhattan(i,j) 

# compute gvalue
gvalue = np.zeros([nrows,ncols]).astype(int)
gvalue = np.array(gvalue).astype(int)
for i in range(nrows):
 for j in range(ncols):
   gvalue[i,j] = 999
gvalue[start[0],start[1]] = 0


# find gopenList, row and column of openList[,]
def find_g_r_c(openList):
  nr = openList.shape[0]
  if (nr == 0):
    print "fail"
    exit()

  g_plus_H = openList.copy()
  for i in range(nr):      
    g_plus_H[i,0] += H[openList[i,1],openList[i,2]]

  # find row corresponding to min gvalue
  idum = (nrows+1)*(ncols+1)
  for i in range(nr):
    if(g_plus_H[i,0] < idum):
      idum = g_plus_H[i,0]
      irow = i
  #irow = g_plus_H[:,0].argmin() 
  return openList[irow,0], openList[irow,1], openList[irow,2], irow  
  

# determine am I inside grid? 
def am_i_in_grid(r,c):
  if (r >= 0 and r <= nrows-1 and c >= 0 and c <= ncols-1):
    return 1
  else:
    return 0

# append row to 2d array
def arr_append(arr,app):
  nr, nc = arr.shape
  arr1 = np.zeros([nr+1,nc]).astype(int)
  for i in range(nr):
   for j in range(nc):
     arr1[i,j] = arr[i,j]
  for j in range(nc):
   arr1[nr,j] = app[j] 
  return arr1


# update one step
def update(g,r1,c1,irow,closedList,openList):
   ig = am_i_in_grid(r1,c1)   
   if (ig == 1):     
     if (grid[r1,c1] == 0 and closedList[r1,c1] == 0):                                
          openList = arr_append(openList,[g+1,r1,c1])
          closedList[r1,c1] = -1          
   return closedList, openList  


# determine if I have reached the end location
def reached_end(openList):
  nr = openList.shape[0]
  reached = 0
  g = 0
  for i in range(nr):
    r = openList[i,1]
    c = openList[i,2]
    if (r == end[0] and c == end[1]):
       reached = 1 
       g = openList[i,0]
  return reached, g



# openList is a 2d array where each row stores [gopenList,row,column]
openList = []
openList.append([0, start[0], start[1]])
openList = np.array(openList).astype(int)

# closedList keeps track of which cells in grid have been visited
# 0 for unvisited cells; -1 for visited cells
closedList = np.zeros([nrows,ncols]).astype(int)
closedList[start[0],start[1]] = -1


# start the main loop

reached = 0
while (reached == 0):  
   g, r, c, irow = find_g_r_c(openList)
   gvalue[r,c] = g
   
   
   # move up
   r1 = r-1; c1 = c
   closedList, openList = update(g,r1,c1,irow,closedList,openList)
     
   # move down
   r1 = r+1; c1 = c
   closedList, openList = update(g,r1,c1,irow,closedList,openList)

   # move left
   r1 = r; c1 = c-1
   closedList, openList = update(g,r1,c1,irow,closedList,openList)   

   # move right
   r1 = r; c1 = c+1   
   closedList, openList = update(g,r1,c1,irow,closedList,openList)

   # delete one row of openList    
   openList = np.delete(openList,(irow),axis=0)

   # have I reached end?
   reached, g = reached_end(openList)
   
   if (reached == 1):
     print "reached end in ", g, " steps"   

gvalue[end[0],end[1]] = g


def backtrack(r,c,r1,c1,g,gvalue,path):
  z = am_i_in_grid(r1,c1)
  if (z == 1):
     if (gvalue[r1,c1] == g-1):
       path[r1,c1] = 1       
       g -= 1
       r = r1
       c = c1
     else:
       z = 0
  return z, g, r, c, path


# find shortest path
path[end[0],end[1]] = 1
reached = 0
r = end[0]
c = end[1]

while (reached == 0):

   # move up
   r1 = r-1; c1 = c   
   z, g, r, c, path = backtrack(r,c,r1,c1,g,gvalue,path)
    

   if (z == 0):
    # move down
    r1 = r+1; c1 = c
    z, g, r, c, path = backtrack(r,c,r1,c1,g,gvalue,path)
    
   if (z == 0):  
    # move left
    r1 = r; c1 = c-1
    z, g, r, c, path = backtrack(r,c,r1,c1,g,gvalue,path) 

   if (z == 0):
    # move right
    r1 = r; c1 = c+1   
    z, g, r, c, path = backtrack(r,c,r1,c1,g,gvalue,path) 
 
   if (z == 0):
     print "not possible!!! "
     exit()
   
   # have I reached start?
   if (g == 1):  
     reached = 1    
 

print 
print "path = "
print path
print "---------------------------------------------------"

