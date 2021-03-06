#!/usr/bin/env python2

import time
import random
import drawSample
import math
import _tkinter
import sys
import imageToRects
import numpy 
import matplotlib.pyplot as plt

try:
    s,obstacles = imageToRects.imageToRects(sys.argv[1])
except:
    print "usage: rrt_planner_line_robot.py <image>"

####################
# CONFIG VARIABLES
####################

visualize = 1
prompt_before_next = 1
SMALLSTEP = 4 
LINESIZE = 50
G = [[0], []]   # nodes, edges
XMAX = s[0]
YMAX = s[1]
       
# goal/target
tx = 800
ty = 150
ta = 0
# start
start_x = 100
start_y = 630
start_a = 0

vertices = [ [start_x,start_y,start_a] ]

sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0
sigmaa_for_randgen = math.pi

nodes=0
edges=1
maxvertex = 0

#####################

def drawGraph(G):
    """ 
        If visualize is turned on, it'll draw all the edges in the tree
    """
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
        if len(vertices)!=1:
            canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


def genPoint():
    """ 
        Generate points using Gaussian dist with mean at goal
        Using just a Gaussian dist is slow, so steer it towards the 
        goal more and every ~5 iterations choose the goal point as
        a 'random' point
    """
    #x = random.gauss(tx, sigmax_for_randgen)
    #y = random.gauss(ty, sigmay_for_randgen)
    #a = random.gauss(ta, sigmaa_for_randgen)

    # As a point of comparison, also use a uniform dist to sample points
    x = random.uniform(0, XMAX)
    y = random.uniform(0, YMAX)
    a = random.uniform(0, math.pi)

    if math.floor(random.randrange(5)) == 4: # lucky number
        x = tx
        y = ty
        a = ta
    return [x, y, a]


def genvertex():
    """
        Append the generated point to the collection of vertices
    """
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    """
        Append the point p to the collection of vertices
    """
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    """
        Return a random vertex in the tree
    """
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    """ 
        Generate all the points in a line from p1 to p2
    """
    line = []
    llsq = 0.0 # line length squared
    for i in range(len(p1)):  # each dimension
        h = p2[i] - p1[i] 
        line.append( h )
        llsq += h*h
    ll = math.sqrt(llsq)  # length
    # normalize line
    if ll <=0: return [0,0]
    for i in range(len(p1)):  # each dimension
        line[i] = line[i]/ll
    return line

def pointPointDistance(p1,p2):
    """ 
        Return the distance between a pair of points (L2 norm). 
    """
    llsq = 0.0 # line length squared
    # faster, only for 2D
    h = p2[0] - p1[0] 
    llsq = llsq + (h*h)
    h = p2[1] - p1[1] 
    llsq = llsq + (h*h)
    return math.sqrt(llsq)

    for i in range(len(p1)):  # each dimension, general case
        h = p2[i] - p1[i] 
        llsq = llsq + (h*h)
    return math.sqrt(llsq)

def closestPointToPoint(G,p2):
    """
        Find the closest point in the tree G to p2, not in G
    """
    dmin = 999999999
    for v in G[nodes]:
        p1 = vertices [ v ]
        d = pointPointDistance(p1,p2)
        if d <= dmin:
            dmin = d
            close = v
    return close

def returnParent(k):
    """ 
        Return parent note for input node k. 
    """
    for e in G[edges]:
        if e[1]==k: 
            if visualize:
                line = (LINESIZE / 2 * math.cos(vertices[e[0]][2]), LINESIZE / 2 * math.sin(vertices[e[0]][2]))
                bot_head = (vertices[e[0]][0] + line[0], vertices[e[0]][1] + line[1])
                bot_tail = (vertices[e[0]][0] - line[0], vertices[e[0]][1] - line[1])
                canvas.polyline(  [bot_head, bot_tail], style=4  )
                canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def pickGvertex():
    """
        Pick a random vertex in the tree G
    """
    try: edge = random.choice( G[edges] )
    except: return pickvertex()
    v = random.choice( edge )
    return v

def redraw():
    """
        Redraw the image, obstacles / tree
    """
    canvas.clear()
    canvas.markit(start_x, start_y, r=SMALLSTEP)
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")

def ccw(A,B,C):
    """ 
        Determine if three points are listed in a counterclockwise order.
        For three points A, B and C. If the slope of the line AB is less than 
        the slope of the line AC then the three points are in counterclockwise order.
        See:  http://compgeom.cs.uiuc.edu/~jeffe/teaching/373/notes/x06-sweepline.pdf
    """
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
    """ 
       Determine if A, B intersect
    """
    i = ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
    return i

def lineHitsRect(p1,p2,r):
    """
        Check if line formed by p1, p2 goes through rectangle r
    """
    rline = ( (r[0],r[1]), (r[0],r[3]) )
    if intersect( p1, p2, rline[0], rline[1] ): return 1
    rline = ( (r[0],r[1]), (r[2],r[1]) )
    if intersect( p1, p2, rline[0], rline[1] ): return 1
    rline = ( (r[0],r[3]), (r[2],r[3]) )
    if intersect( p1, p2, rline[0], rline[1] ): return 1
    rline = ( (r[2],r[1]), (r[2],r[3]) )
    if intersect( p1, p2, rline[0], rline[1] ): return 1
    return 0

def inRect(p,rect,dilation):
    """ 
        Return 1 in p is inside rect, dilated by dilation (for edge cases). 
    """
    if p[0]<rect[0]-dilation: return 0
    if p[1]<rect[1]-dilation: return 0
    if p[0]>rect[2]+dilation: return 0
    if p[1]>rect[3]+dilation: return 0
    return 1

def rrt_search(G, tx, ty, ta):
    """
     While goal hasn't been reached yet:
        sample a random point
        determine the nearest point on the tree to it
        determine if a line can be drawn from the nearest point to the new
        without running into an obstacle
        if it can, add it to the tree
        if goal hasn't been reached yet, repeat
    """

    dist_from_goal = 200
    iteration = 0
    drawGraph(G)
    while dist_from_goal > SMALLSTEP:
        iteration += 1
        rand_point = genPoint()
        near_point = closestPointToPoint(G,rand_point)
        near_point_v = vertices[near_point]

        # Ensure that the line robot 'steers' towards the closest angle
        if rand_point[2] - near_point_v[2] > (math.pi / 2):
            rand_point[2] -= math.pi
        elif near_point_v[2] - rand_point[2] > (math.pi / 2):
            rand_point[2] += math.pi

        line_to_point = lineFromPoints(near_point_v, rand_point)
        new_point = [ near_point_v[0] + line_to_point[0] * SMALLSTEP,\
                      near_point_v[1] + line_to_point[1] * SMALLSTEP,\
                     (near_point_v[2] + line_to_point[2] * SMALLSTEP) % math.pi ]

        in_boundary = True
        if new_point[0] >= XMAX or new_point[0] < 1 or new_point[1] >= YMAX or new_point[1] < 1:
            in_boundary = False
        else: 
            for obstacle in obstacles:
                line = (LINESIZE/2*math.cos(new_point[2]), LINESIZE/2*math.sin(new_point[2]))
                bot_tail = (new_point[0] - line[0], new_point[1] - line[1])
                bot_head = (new_point[0] + line[0], new_point[1] + line[1])

                if lineHitsRect(bot_tail, bot_head, obstacle) or \
                   inRect(bot_head, obstacle, 0.1) or inRect(bot_tail, obstacle, 0.1):
                    in_boundary = False
                    break

        if in_boundary:
            new_vertex = pointToVertex(new_point)
            G[nodes].append(new_vertex)
            G[edges].append((near_point, new_vertex))
            if visualize:
                canvas.polyline([vertices[near_point], vertices[new_vertex]])
                canvas.events() 
            if (tx - SMALLSTEP <= new_point[0]) and (tx + SMALLSTEP >= new_point[0]) and (ty - SMALLSTEP <= new_point[1]) and (ty + SMALLSTEP >= new_point[1]):
                graph_node = len(vertices) - 1
                path_dist = 0
                while (graph_node != 0):
                    graph_node = returnParent(graph_node)
                    path_dist += 1
                return (iteration, path_dist) 

    graph_node = len(vertices) - 1 
    path_dist = 0 
    while (graph_node != 0): 
        graph_node = returnParent(graph_node)
        path_dist += 1
    return (iteration, path_dist)


if visualize:
    canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)

if visualize:
    for o in obstacles: canvas.showRect(o, outline='red', fill='blue')

maxvertex += 1

# if rrt_search worked correctly, it only needs to run once to 
# find the goal 

G = [  [ 0 ]  , [] ]   # nodes, edges
vertices = [ [start_x, start_y, start_a] ]
redraw()

if visualize: canvas.markit( tx, ty, r=SMALLSTEP )

drawGraph(G)
rrt_search(G, tx, ty, ta)

if visualize:
    canvas.mainloop()
