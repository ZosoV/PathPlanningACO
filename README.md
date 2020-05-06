# PathPlanningACO
This project shows the design of a path planning algorithm based on the Ant Colony Optimization (ACO) algorithm. The project is composed of several folders and scripts where we can find some ACO versions and other path planning methods.

## Objective

Find the shortest path from a initial to a final node in three-dimensional terrain (built with graph) using the Ant Colony Optimization algorithm.

## How to explore the project
### Environment Problem 

The environment is in the folder `.\EnvironmentProblem` where the scripts defined a weighted graph G(N,E) composed by Node and Edge classes. The nodes of the graph are loaded from `.obj` files that are in the folder `./PathPlanningACO/bin/Debug/netcoreapp2.1/data_sets/`. We generate four types of terrains: double valley, valley, mountain and a perlin noise terrains. This same configuration of the environment is used for all the methods.
```
\EnvironmentProblem
  Edge.cs
  MeshEnvironment.cs
  Node.cs
```
### ACO

The main algorithm is in the folder `.\ACO`. Her, we can find 4 scripts. The basic algorithm is the `AntColonyOptimizationv0.cs` that used a `Antv0` class for solving the path planning problem. Furtheremore, the scripts `RandomWalkv1.cs` and `RandomWalkv2.cs` are two proposed random walks that are used to improve the performance of the algorithm in huge graphs.
```
\ACO
  AntColonyOptimizationv0.cs
  Antv0.cs
  RandomWalkv1.cs
  RandomWalkv2.cs
```
### Other Methods
The folder `.\OtherMethods` contains three different path planning methods: `A*`, `Dijkstra`, `Genetic` algorithms. The three methods works under the same environment.
```
\OtherMethods
  \A_star
    AStarAlgorithm.cs
    Tag.cs
  \Dijkstra
    DijkstraAlgorithm.cs
    DijkstraJustCost.cs
  \Genetic
    GeneticAlgorithm.cs
    Individual.cs
````
### Testing
The folder `.\Testing` contains some function for testing the algorithms. First, the `TestACO.cs` have the function that comprobe the functionability of ACOv0 alone.

```
\Testing
    MeasureFunctions.cs
    TestACO.cs
    TestRandomWalksACO.cs
    TestSeveralMethods.cs
````
