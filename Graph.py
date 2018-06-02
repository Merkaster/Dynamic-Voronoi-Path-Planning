import sys
import numpy as np
from collections import defaultdict

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited        
        self.visited = False  
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return

import heapq

def planning(start,end,vor):


	startClosest =[]
	endClosest =[]
	startMinDistance =100000000000
	endMinDistance = 1000000000000
	start = np.asarray(start)
	end = np.asarray(end)
	vertices = defaultdict()
	
	#Find the closest voronoi nodes from the starting position and the target position
	for i in vor.vertices:

		sDist = np.linalg.norm(start-i)
		eDist = np.linalg.norm(end-i)

		if sDist < startMinDistance:
			startMinDistance = sDist
			startClosest = i
		if eDist < endMinDistance:
			
			endMinDistance = eDist
			endClosest = i

	#Create a graph with nodes the voronoi nodes, the starting position and the target position,Create a dictionary with keys a number and values a point (x,y)
	#e.g Graph with 20 nodes, vertices[0] = starting point, vertices[19] = target point
	# Create an edge from starting position to the closest voronoi node
	# Create an edge from the target position to the closest voronoi node
	g = Graph()

	counter =1
	startKey =0
	endKey =0
	for vertex in vor.vertices:

		vertices[counter] = vertex

		if np.array_equal(vertices[counter],startClosest):
			startKey = counter
		if np.array_equal(vertices[counter],endClosest):
			endKey = counter
		g.add_vertex(counter)
		counter = counter+1

	vertices[0] = start
	vertices[counter] = end

	g.add_vertex(str(0))
	g.add_vertex(str(counter))

	g.add_edge(str(0), str(startKey), 0)
	g.add_edge(str(counter), str(endKey), 0)

	#Edges from the voronoi diagram add them as edges to the Graph
	for vpair in vor.ridge_vertices:
		if vpair[0] >=0 and vpair[1] >=0:
			v0 = vor.vertices[vpair[0]]
			v1 = vor.vertices[vpair[1]]
			for key,value in vertices.items():
				if np.array_equal(v0,value):
					key1 = key
				if np.array_equal(v1,value):
					key2 = key

			
			g.add_edge(str(key1), str(key2), 0)
	
	
	#Perform dijkstra
	dijkstra(g, g.get_vertex(str(0)))
	target = g.get_vertex(str(len(vertices.keys())-1))
	
	path = [target.get_id()]
	shortest(target, path)
	
	# return path
	return path[::-1],vertices

def dijkstra(aGraph, start):
    print ('Calculating Dijkstras shortest path ')
    # Set the distance for the start node to zero 
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance 
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)

