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
import sys


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
        if a not in self.list_vertices.keys():
            nv_a = self.add_vertex(a)
        if b not in self.list_vertices.keys():
            nv_b = self.add_vertex(b)
        self.list_vertices[a].add_neighbour(self.list_vertices[b].id, int(weight))
        self.count_edges +=1
        if not directed:
            self.list_vertices[b].add_neighbour(self.list_vertices[a].id, int(weight))
            self.count_edges +=1
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
