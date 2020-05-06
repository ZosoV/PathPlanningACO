using System;
using System.Collections.Generic;
using System.Text;

namespace PathPlanningACO.OtherMethods.Dijkstra
{
    class DijkstraJustCost
    {
        public Double execution_time = 0;
        private static int MinimumDistance(Double[] distance, bool[] shortestPathTreeSet, int verticesCount)
        {
            Double min = Double.MaxValue;
            int minIndex = 0;

            for (int v = 0; v < verticesCount; ++v)
            {
                if (shortestPathTreeSet[v] == false && distance[v] <= min)
                {
                    min = distance[v];
                    minIndex = v;
                }
            }

            return minIndex;
        }

        private static void Print(Double[] distance, int verticesCount)
        {
            Console.WriteLine("Vertex    Distance from source");

            for (int i = 0; i < verticesCount; ++i)
                Console.WriteLine("{0}\t  {1}", i, distance[i]);
        }

        public Double DijkstraAlgo(Double[,] graph, int source, int verticesCount)
        {
            //Time variable
            var watch = System.Diagnostics.Stopwatch.StartNew();

            Double[] distance = new Double[verticesCount];
            bool[] shortestPathTreeSet = new bool[verticesCount];

            for (int i = 0; i < verticesCount; ++i)
            {
                distance[i] = int.MaxValue;
                shortestPathTreeSet[i] = false;
            }

            distance[source] = 0;

            for (int count = 0; count < verticesCount - 1; ++count)
            {
                int u = MinimumDistance(distance, shortestPathTreeSet, verticesCount);
                shortestPathTreeSet[u] = true;

                for (int v = 0; v < verticesCount; ++v)
                    if (!shortestPathTreeSet[v] && Convert.ToBoolean(graph[u, v]) && distance[u] != int.MaxValue && distance[u] + graph[u, v] < distance[v])
                        distance[v] = distance[u] + graph[u, v];
            }

            watch.Stop();
            execution_time = watch.ElapsedMilliseconds;
            //Print(distance, verticesCount);

            return Math.Round(distance[verticesCount - 1], 2);
        }
    }

}
