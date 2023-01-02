# SmartPuzzleSolver
Code that solve N-puzzle game using BFS, DFS, or A* algorithms.

## Description 🧐

This is a program that designed to solve N-puzzle game through three well known Algorithms:
* Breath-First-Search Algorithm ( Uninformed Search )
* Depth-First-Search Algorithm ( Uninformed Search )
* A* Algorithm ( Informed Search )

This program shows the follwing:
* Path to the goal
* Cost of that path
* Number of expanded nodes
* Depth of the search
* Time and Memory Consumtion

## Getting Started

### Dependencies

* Language/Version: python 3.9
* Libraries: sys, math, time, heapq, resource, collections(deque)

### Executing program

* The program will be executed in the following format: 
```
$ python3 puzzle.py <method> <board>
```
* Example:
```
$ python3 puzzle.py ast 1,3,4,5,7,0,8,2
```

## Acknowledgments

* This program have been inspired by one of the projects in AI class.

