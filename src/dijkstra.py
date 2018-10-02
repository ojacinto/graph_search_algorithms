#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------
# Copyright 2018(c). All rights reserved.
#
# This is free software; you can do what the LICENCE file allows you to.
# Author: Ing. Oraldo Jacinto Simon

import sys
import heapq

##############################################################################
#Dijkstra
##############################################################################
def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.predecessor:
        path.append(v.predecessor.get_id())
        shortest(v.predecessor, path)
    return

def dijkstra_search(Graph, start, target):
    print('''Dijkstra's shortest path''')
    start = Graph.get_vertex(start)
    target = Graph.get_vertex(target)
    # Set the distance for the start node to zero
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in Graph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()
        #for next in v.adjacent:
        for prox, weight in current.adjacency.items():
            # if visited, skip
            prox = Graph.get_vertex(prox)
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
        unvisited_queue = [(v.get_distance(),v) for v in Graph if not v.visited]
        heapq.heapify(unvisited_queue)


    path = [target.get_id()]
    shortest(target, path)
    print ('The shortest path : %s' %(path[::-1]))
