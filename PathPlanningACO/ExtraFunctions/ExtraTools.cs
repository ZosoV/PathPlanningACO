using PathPlanningACO.EnvironmentProblem;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathPlanningACO.ExtraFunctions
{
    class ExtraTools
    {
        public static string PrintList(ref List<int> list)
        {
            string str = "[" + list[0];

            for (int i = 1; i < list.Count; i++)
            {
                str += "," + list[i];
            }
            str += "]";

            return str;

        }
        //-------------------------------------------------------------------------------

        public static Double EucDistance(Node point1, Node point2)
        {
            return Math.Sqrt((Double)Math.Pow(point1.X - point2.X, 2) + (Double)Math.Pow(point1.Y - point2.Y, 2) + (Double)Math.Pow(point1.Z - point2.Z, 2));
        }
        //-------------------------------------------------------------------------------
        public static Double NormalizeWithInterval(Double value, Double min_value, Double max_value)
        {
            //Los valores de visibilidad van de 0 a 3

            //Normalize to [0,1]
            Double range = max_value - min_value;
            Double result = (value - min_value) / range;

            return result;
        }


        //-------------------------------------------------------------------------------
        //Funcion de coste del camino actual
        public static Double GetCost(ref List<int> path, ref MeshEnvironment env)
        {
            Double cost = 0;
            for (int i = 0; i < path.Count - 1; i++)
            {
                int node1_idx = path[i];
                int node2_idx = path[i + 1];

                int index = env.world[node1_idx].neighboors.IndexOf(node2_idx);

                int edge_idx = env.world[node1_idx].edges[index];

                cost += env.edges[edge_idx].distance;
            }
            return cost;
        }

        //------------------------------------------------------------------
        public static Double GetCostJoinRW_KN(ref List<int> path, ref MeshEnvironment env)
        {
            Double cost = 0;
            for (int i = 0; i < path.Count - 1; i++)
            {
                int node1_idx = path[i];
                int node2_idx = path[i + 1];

                int index = env.world[node1_idx].neighboors.IndexOf(node2_idx);

                int edge_idx = env.world[node1_idx].edges[index];

                cost += env.edges.Find(e => e.id == edge_idx).distance;
            }
            return cost;
        }
        //-------------------------------------------------------------------------------
        public static void PrintRoute(ref List<int> route)
        {
            foreach (var node in route)
            {
                Console.Write("| {0} |", node);
            }
            Console.WriteLine("");
        }

        //---------------------------------------------------------------
        public static IEnumerable<T> Randomize<T>(IEnumerable<T> source)
        {
            Random rnd = new Random();
            return source.OrderBy<T, int>((item) => rnd.Next());
        }

    }
}
