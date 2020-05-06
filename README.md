# PathPlanningACO
This project shows the design of a path planning algorithm based on the Ant Colony Optimization (ACO) algorithm. The project is composed of several folders and scripts where we can find some ACO versions and other path planning methods.

## Objective

Find the shortest path from a initial to a final node in three-dimensional terrain (built with graph) using the Ant Colony Optimization algorithm.

## How to explore the project
### Environment Problem 

The environment is in the folder `./EnvironmentProblem` where the scripts defined a weighted graph G(N,E) composed by Node and Edge classes. This environment is used for all the methods.

```
\EnvironmentProblem
  Edge.cs
  MeshEnvironment.cs
  Node.cs
```

### ACO

The main algorithm is in the folder `./ACO`. Her, we can find 4 scripts. The basic algorithm is the `AntColonyOptimizationv0.cs` that used a `Antv0` class for solving the path planning problem. Furtheremore, the scripts `RandomWalkv1.cs` and `RandomWalkv2.cs` are two proposed random walks that are used to improve the performance of the algorithm in huge graphs.

```
\ACO
  AntColonyOptimizationv0.cs
  Antv0.cs
  RandomWalkv1.cs
  RandomWalkv2.cs
```

### Other Methods
