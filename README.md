# PathPlanningACO
This project shows the design of a path planning algorithm based on the Ant Colony Optimization (ACO) algorithm. The project is composed of several folders and scripts where we can find some ACO versions and other path planning methods. The information of this project is described in more detail in the following link: [The Blog will come soon]()

Note: To review an extended and updated version of this work explore the following repository [aco_random_walk](https://github.com/ZosoV/aco_random_walk).
## Objective

Find a feasible path from an initial to a final node in a three-dimensional terrain (represented with graph) using the Ant Colony Optimization algorithm.

## Simulation Video

[https://www.youtube.com/watch?v=c0dzv3qGqrs&t=114s](https://www.youtube.com/watch?v=c0dzv3qGqrs&t=114s)

## How to explore the project
### Environment Problem 

The environment is in the folder `./EnvironmentProblem` where the scripts defined a weighted graph G(N,E) composed by Node and Edge classes. The nodes of the graph are loaded from `.obj` files that are in the folder `./PathPlanningACO/bin/Debug/netcoreapp2.1/data_sets/`. We generate four types of terrains: double valley, valley, mountain and a perlin noise terrains. The same configuration of the environment is used for all the methods presented in this work. 

You can use your own `.obj` files. To this end, you have to copy the file in the folder `data_sets` and configure the function `InitEnvironment` from the `MeshEnvironment` class.
```
/EnvironmentProblem
  Edge.cs
  MeshEnvironment.cs
  Node.cs
```
### ACO

The main algorithm is in the folder `./ACO`. Here, we can find four scripts. The standard algorithm is the `AntColonyOptimizationv0.cs` that used an `Antv0` class for solving the path planning problem. We labelled to this standard algorithm as `ACOv0`. Furthermore, the scripts `RandomWalkv1.cs` and `RandomWalkv2.cs` are two proposed random walks that are preprocessing algorithm, used to improve the performance of the standard algorithm in huge graphs.
```
/ACO
  AntColonyOptimizationv0.cs
  Antv0.cs
  RandomWalkv1.cs
  RandomWalkv2.cs
```
### Other Methods
The folder `./OtherMethods` contains three different path planning methods: `A*`, `Dijkstra`, `Genetic` algorithms. The three methods work under the same environment. They were used to compare our proposal.
```
/OtherMethods
  /A_star
    AStarAlgorithm.cs
    Tag.cs
  /Dijkstra
    DijkstraAlgorithm.cs
    DijkstraJustCost.cs
  /Genetic
    GeneticAlgorithm.cs
    Individual.cs
````
### Testing
The folder `./Testing` contains some functions for testing the algorithms. In this project, we perform two tests:

1. Performance evaluation of `ACOv0` alone, `RW1 + ACOv0` and `RW2 + ACOv0`. To this end, we used the scripts: `TestACO.cs` and `TestRandomWalks.cs`.
2. Performance evaluation of the best ACO variant with other methods. To this end, we used as the best performance version `RW2 + ACOv0` and compared with `A*`, `Dijkstra`, and `Genetic` algorithms

Each test was executed `n=30` times per each terrain and each size of the mesh. Some testing examples can be found in the script `Program.cs` in the root folder PathPlanningACO.

```
/Testing
    MeasureFunctions.cs
    TestACO.cs
    TestRandomWalksACO.cs
    TestSeveralMethods.cs
````

Usage is as follows:

1. Change the parameters in the attributes of the selected class. 
2. Instantiate and initialize the environment class.
3. Instantiate the respective class with the environment, and execute the function.
