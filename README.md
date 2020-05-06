# PathPlanningACO
This project shows the design of a path planning algorithm based on the Ant Colony Optimization algorithm. The project is composed of several folders and scripts where we can find some ACO versions and other path planning methods.

## Objective


## How to explore the project
The project is composed of the following files: 
```
\ACO
  AntColonyOptimizationv0.cs
  Antv0.cs
  RandomWalkv1.cs
  RandomWalkv2.cs
\EnvironmentProblem
  Edge.cs
  MeshEnvironment.cs
  Node.cs
\ExtraFunctions
  ExtraTools.cs
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
\Testing
    MeasureFunctions.cs
    TestACO.cs
    TestRandomWalksACO.cs
    TestSeveralMethods.cs
Program.cs
```

### Environment Problem 

The environment is defined in the folder `./EnvironmentProblem` where the scripts defined a weighted graph G(N,E).

### ACO

