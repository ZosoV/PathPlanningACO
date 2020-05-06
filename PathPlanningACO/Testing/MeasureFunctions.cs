using PathPlanningACO.EnvironmentProblem;
using System;
using System.Collections.Generic;
using System.Text;

namespace PathPlanningACO.Testing
{
    class MeasureFunctions
    {
        //--------------------------------------------------------------------
        public static int GetVisitedEdges(ref MeshEnvironment env)
        {
            int visited_edges = 0;

            foreach (var edge in env.edges)
            {
                if (edge.visited)
                {
                    visited_edges++;
                }
            }

            return visited_edges;
        }

        //--------------------------------------------------------------------
        public static int GetVisitedNodes(ref MeshEnvironment env)
        {
            int visited_nodes = 0;

            foreach (var node in env.world)
            {
                if (node.visited)
                {
                    visited_nodes++;
                }
            }

            return visited_nodes;
        }
        //--------------------------------------------------------------------

        private static int SelectNextNode(ref MeshEnvironment env, ref List<int> current_route, int current_node)
        {
            //Get the info the current node: neighboors node, edges, and proximities
            List<int> possible_next_nodes = new List<int>(env.world[current_node].neighboors);
            List<int> possible_next_edges = new List<int>(env.world[current_node].edges);

            int next_node = -1;

            if (possible_next_nodes.Count == 1 && !current_route.Contains(possible_next_nodes[0]))
            {
                next_node = possible_next_nodes[0];
            }
            else
            {
                Double accu_pheromone = 0;
                Double max_pheromone = Double.MinValue;
                for (int i = 0; i < possible_next_nodes.Count; i++)
                {
                    int node_idx = possible_next_nodes[i];
                    int edge_idx = possible_next_edges[i];
                    Double current_pheromone = env.edges[edge_idx].pheromone_amount;

                    if (current_pheromone > max_pheromone && !current_route.Contains(node_idx))
                    {
                        max_pheromone = current_pheromone;
                        next_node = node_idx;
                    }

                    if (!current_route.Contains(node_idx))
                    {
                        accu_pheromone += current_pheromone;
                    }
                }

                if (accu_pheromone == 0)
                {
                    next_node = -1;
                }
            }

            return next_node;
        }

        //--------------------------------------------------------------------
        private static List<int> GetRoute(ref MeshEnvironment env, int initial_node)
        {
            List<int> route = new List<int>();
            route.Add(initial_node);

            //Variable para detectar si ha encontrada una ruta valida
            bool found_route = false;

            int current_node = initial_node;
            while (!found_route)
            {
                int next_node = SelectNextNode(ref env, ref route, current_node);
                if (next_node != -1)
                {

                    //Añado el nuevo nodo y continuo en la sig. posicion
                    route.Add(next_node);
                    current_node = next_node;
                }
                else
                {
                    //Si se produce una condicion de atasque salgo del bucle
                    break;
                }

                //Configuro mi variable si he encontrado el objetivo
                found_route = current_node == env.final_node;
            }

            if (!found_route)
            {
                route = new List<int>();
            }

            return route;
        }
        //--------------------------------------------------------------------
        public static Double CalculatePercentageLearning(ref MeshEnvironment env)
        {
            int learning_positions = 0;

            for (int i = 0; i < env.world.Count - 1; i++)
            {

                List<int> route = GetRoute(ref env, i);

                if (route.Count != 0)
                {
                    learning_positions++;
                }

            }

            Double percentage = Math.Round((Double)learning_positions * 100 / (Double)(env.world.Count - 1), 2);

            return percentage;
        }
    }
}
