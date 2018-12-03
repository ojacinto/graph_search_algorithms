
`Search algorithms for graphs: (Search in width, depth, dijikstra and kruskal)`
===============================================================================

The Operations Graphs are a data structure that serves to model an infinity of
problems that can be expressed in a computational way. Unlike the tree
structure, graphs are not a rigid structure (but much more flexible), which
allows them to be used in applications such as: maps, distributed computing,
social networks, people mobility, database, intelligence artificial ect.

This structure facilitates the solution of many problems and is widely used in
software applications. In the present work we present the search algorithms in
width, depth and Dijkstra (shortest path) on graph theory. These algorithms
are implemented (python) in a console application that loads two test graphs
that are found in txt extension files.


To solve the implementation of these algorithms, the classes were implemented:
vertex and graph contained in the python ``graph.py`` module. As part of the graph
class, the width search and depth search algorithms were implemented. The
dijkstra algorithm was implemented in a separate module with the same name,
although it could be included as part of the class graph without any problem
for future work.

`graph1.txt`
===========
This graph is used to test the methods of width search and depth search.

`graph2.txt`
===========
This graph is used to test the dijkstra method (shortest path) and
the search for the minimum expansion tree (Kruskal)

As a controller, the main.py module is used, which prints the results of the three algorithms in the console.

Note :
======
In the case of graph1, the search path in width and depth is calculated from
the vertex node `a`.

In the case of graph2, the shortest path is calculated using the dijkstra
solution from vertex "a" to "e".
