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

from collections import deque
from graph import Graph, sys, heapq
from texttable import Texttable

class Main(object):

    def __init__(self):
        '''Search algorithms, depth search, width search and minimum path using Dijkstra

        '''
        print ('''*******************Search algorithms********************
               Author: Oraldo Jacinto Simon
               Professor: M.I. Jesus Roberto López Santillán
               ''')
        list_edges_weight = []
        fichero1 = open("graph1.txt", 'r')
        fichero2 = open("graph2.txt", 'r')
        lines1 = fichero1.read()
        lines2 = fichero2.read()
        lines1 = lines1.rsplit('\n')
        lines2 = lines2.rsplit('\n')
        list_edges_weight1 = [edge.split(',') for edge in lines1 if edge !='']
        list_edges_weight2 = [edge.split(',') for edge in lines2 if edge !='']
        fichero1.close()
        fichero2.close()
        obj_graph1 = Graph()
        obj_graph2 = Graph()
        graph1 = obj_graph1.add_from_edge(list_edges_weight1, directed=True)
        adjacents = graph1.get_adjacents()
        width = graph1.search_in_width('a');
        width_recursive = graph1.search_width_recursive('a', queue=deque(), visited=[]);
        depth = graph1.depth_search('a')
        graph2 = obj_graph2.add_from_edge(list_edges_weight2, directed=False)
        adjacents2 = graph2.get_adjacents()

        table = Texttable()
        table.set_cols_align(["c", "c", "c"])
        table.set_cols_valign(["t", "m", "b"])
        table.set_cols_width([20,70,40])
        head = ["Method", "Graph", "Answer"]
        rows = []
        rows.append(head)
        search_in_width = [
            'Search in width from the vertex (a)',
            str(adjacents),
            str(width_recursive)
        ]
        rows.append(search_in_width)
        depth_search = [
            'Depth Search from the vertex (a)',
            str(adjacents),
            str(depth)
        ]
        rows.append(depth_search)
        dijkstra = [
            'Dijkstra',
            str(adjacents2),
            'See trace below'
        ]
        rows.append(dijkstra)
        table.add_rows(rows)
        print(table.draw() + "\n")
        graph2.dijkstra_search('a', 'e')

Main()
