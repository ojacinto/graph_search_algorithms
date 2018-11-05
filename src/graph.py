#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------
# Copyright 2018(c). All rights reserved.
#
# This is free software; you can do what the LICENCE file allows you to.
# Author: Ing. Oraldo Jacinto Simon

from __future__ import (division as _py3_division,
                        print_function as _py3_print,
                        absolute_import as _py3_abs_import)

from collections import deque, OrderedDict
import sys, heapq

parent_aux = dict()
rank = dict()

class Vertex(object):
    '''Represents a vertex of a graph

    '''

    def __init__(self, name):
        self.id = name
        self.visited = False
        self.adjacency = OrderedDict()
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited
        self.visited = False
        # Predecessor
        self.predecessor = None


    def __str__(self):
        adjacents = [(self.id+v, w) for v, w in self.adjacency.items()]
        return {self.id: adjacents}

    def add_neighbour(self, neighbour, weight=0):
        '''Add a connection from one vertex to another

        '''
        self.adjacency[neighbour] = weight

    def get_id(self):
        '''Get the id's vertex

        '''
        return self.id

    def get_weight(self, vertex):
        '''Returns the weighting of the edge's vertex passed as a parameter.

        '''
        return self.adjacency[vertex] if vertex in self.adjacency.keys() else None

    def get_vertex_adjacents(self):
        '''Returns the adjacent vertices

        '''
        return self.adjacency.keys()

    def get_distance(self):
        '''Get the distance's vertex

        '''
        return self.distance

    def set_visited(self):
        '''Set the distance's vertex

        '''
        self.visited = True

    def set_distance(self, dist):
        self.distance = dist

    def set_previous(self, prev):
        '''Set the predecessor's vertex

        '''
        self.predecessor = prev


class Graph(object):
    '''Class graph

    '''
    def __init__(self):
        self.list_vertices = OrderedDict()
        self.count_vertices = 0
        self.edges = []
        self.count_edges = 0

    def __contains__(self, vertex):
        return vertex in self.list_vertices.keys()

    def __iter__(self):
        return iter(self.list_vertices.values())

    def add_vertex(self, name):
        '''Add a vertex to the graph

        '''
        new_vertex = Vertex(name)
        self.list_vertices[new_vertex.id] = new_vertex
        self.count_vertices += 1
        return new_vertex

    def get_vertex(self, vertex):
        '''Returns the vertice passed by parameter if it exists in the graph
        else return None

        '''
        return self.list_vertices[vertex] if vertex in self.list_vertices.keys() else None

    def addEdge(self, a, b, weight=0, directed=None):
        '''Add an edge to the graph and update the vertex adjacency list.

        '''
        weight = int(weight)
        if a not in self.list_vertices.keys():
            nv_a = self.add_vertex(a)
        if b not in self.list_vertices.keys():
            nv_b = self.add_vertex(b)
        self.list_vertices[a].add_neighbour(self.list_vertices[b].id, weight)
        self.count_edges +=1
        if not directed:
            self.list_vertices[b].add_neighbour(self.list_vertices[a].id, weight)
            self.count_edges +=1
        edge = (weight, a, b)
        self.edges.append(edge)
        return self

    def get_vertices(self):
        '''Returns a list with the vertices of the graph

        '''
        return self.list_vertices.keys()

    def add_from_edge(self, add_edges, directed):
        '''Add edges from a list of tuples

        '''
        for edge in add_edges:
            # (a,b,c) ab=>vertices, c=>weight
            if len(edge) == 3:
                self.addEdge(edge[0], edge[1], edge[2], directed)
            else:
                print ("Format incorrect in graph_example.txt")
        return self

    def get_adjacents(self):
        '''Returns a dictionary with all the vertices and their adjacency lists

        '''
        adjacency = []
        for vertex in self.list_vertices.values():
            adjacency.append(vertex.__str__())
        return adjacency

##############################################################################
# Search in width:
##############################################################################

    def search_in_width(self, node_origin):
        '''Traverses the graph in depth

        Use an auxiliary queue where are add all the adjacent vertices
        Take advantage of the order of the queue to get the vertex when visited
        This way when the queue remains empty is because all the graphs were scanned


        '''
        queue = deque()
        visited = []
        vertex = self.get_vertex(node_origin)
        if vertex is not None:
            queue.append(vertex)
        while len(queue) != 0:
            actual = queue.popleft()
            if actual not in visited:
                visited.append(actual.id)
            if actual.adjacency:
                for adj in actual.adjacency.keys():
                    if adj not in visited:
                        queue.append(self.get_vertex(adj))
        return visited

    def search_width_recursive(self, node_origin, queue=deque(), visited=[]):
        '''Traverses the graph recursively in depth

        Stop condition: If there are visited graphs and in the adjacent queue
        the length is zero, because there are no graphs left to visit

        '''
        # Stop condition
        if visited and len(queue) == 0:
            return visited
        vertex = self.get_vertex(node_origin)
        if vertex is not None and vertex.id not in visited:
            visited.append(vertex.id)
            adjacents = vertex.adjacency.keys()
            if(len(adjacents) > 0):
                queue.extend(adjacents)
        if len(queue) != 0 :
            return self.search_width_recursive(queue.popleft(), queue, visited)

##############################################################################
# Depth search:
##############################################################################
    def depth_search(self, node_origin):
        ''' Go through the graph in depth

        1-The source vertex is placed in a stack
        2-While the stack is not empty
          -If the vertex has not been visited, it is added to the visited list
          -Each of the vertices is reviewed by the adjacency list and is not
           in the visited list: it is added

        '''
        visited = []
        stack = []
        vertex = self.get_vertex(node_origin)
        if vertex is not None:
            stack.append(vertex)
        while stack:
            actual = stack.pop()
            if actual not in visited:
                visited.append(actual.id)
            if actual.adjacency:
                for adj in actual.adjacency.keys():
                    if adj not in visited:
                        stack.append(self.get_vertex(adj))
        return visited

##############################################################################
#Dijkstra
##############################################################################
    def shortest(self, v, path):
        ''' Make shortest path from v.previous'''
        if v.predecessor:
            path.append(v.predecessor.get_id())
            self.shortest(v.predecessor, path)
        return

    def dijkstra_search(self, start, target):
        print('''Dijkstra's shortest path''')
        start = self.get_vertex(start)
        target = self.get_vertex(target)
        # Set the distance for the start node to zero
        start.set_distance(0)

        # Put tuple pair into the priority queue
        unvisited_queue = [(v.get_distance(),v) for v in self]
        heapq.heapify(unvisited_queue)

        while len(unvisited_queue):
            # Pops a vertex with the smallest distance
            uv = heapq.heappop(unvisited_queue)
            current = uv[1]
            current.set_visited()
            #for next in v.adjacent:
            for prox, weight in current.adjacency.items():
                # if visited, skip
                prox = self.get_vertex(prox)
                if prox.visited:
                    continue
                new_dist = current.get_distance() + weight

                if new_dist < prox.get_distance():
                    prox.set_distance(new_dist)
                    prox.set_previous(current)
                    print ('updated : current = %s next = %s new_dist = %s' %(current.get_id(), prox.get_id(), prox.get_distance()))
                else:
                    print ('not updated : current = %s next = %s new_dist = %s' %(current.get_id(), prox.get_id(), prox.get_distance()))

            # Rebuild heap
            # 1. Pop every item
            while len(unvisited_queue):
                heapq.heappop(unvisited_queue)
            # 2. Put all vertices not visited into the queue
            unvisited_queue = [(v.get_distance(),v) for v in self if not v.visited]
            heapq.heapify(unvisited_queue)


        path = [target.get_id()]
        self.shortest(target, path)
        print ('The shortest path : %s' %(path[::-1]))

##############################################################################
#Kruskal
##############################################################################
    def make_set(self, vertex):
        '''Init the structure parent_aux and rank

        '''
        parent_aux[vertex] = vertex
        rank[vertex] = 0

    def find(self, vertex):
        '''Find the root of the vertice passed by parameter within list parent_aux

        '''
        if parent_aux[vertex] != vertex:
            parent_aux[vertex] = self.find(parent_aux[vertex])
        return parent_aux[vertex]

    def union(self, vertex_a, vertex_b):
        '''Allows connecting 2 related components, this is done by the
        following:
         1- Get the root of the vertex a.
         2- Get the root of the vertex b.
         3- Update the father of some of the roots, assigning him as the new
            father the other root.

        '''
        root_a = self.find(vertex_a)
        root_b = self.find(vertex_b)
        if root_a != root_b:
            if rank[root_a] > rank[root_b]:
                parent_aux[root_b] = root_a
	    else:
	        parent_aux[root_a] = root_b
	if rank[root_a] == rank[root_b]: rank[root_b] += 1

    def kruskal(self):
        '''Find the Minimum Spanning Tree (MST) in the graph, calculate the
        cost and verify if the MST is valid.

        The problem of finding the Minimum Spanning Tree (MST) can be solved with
        several algorithms, the most known with Prim and Kruskal both use
        greedy techniques (greedy).

        '''
        mst_cost = 0
        # Minimum Spanning Tree
        mst = set()
        self.edges.sort()
        [self.make_set(v) for v in self.list_vertices.keys()]
        for edge in self.edges:
            weight, vertex_a, vertex_b = edge
            if self.find(vertex_a) != self.find(vertex_b):
                self.union(vertex_a, vertex_b)
                mst.add(edge)
                mst_cost += weight
        # The MST is valid if the number of edges must be equal to the number
        # of vertices - 1. This is true because the MST must have all the
        # vertices of the graph entered and in addition there must be no cycles
        valid = True if len(mst) == self.count_vertices - 1 else False
        return {'mst': sorted(mst), 'cost': mst_cost, 'valid': valid}
