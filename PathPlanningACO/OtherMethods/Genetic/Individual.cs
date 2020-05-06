using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.Linq;

namespace PathPlanningACO.OtherMethods.Genetic
{
    class Individual : IEquatable<Individual>
    {
        //Valor que simboliza qué tan apto es este individuo.
        public Double fitness;
        public Double best_cost;

        //El cromosama es el camino que esta representado por una lista de enteros con los indices de cada nodo
        public List<int> path = new List<int>();

        //Heuristic Extra Information
        public bool is_feasible;
        public int num_infeasible_nodes; // 0 does not have infeasible nodes
        public int num_infeasible_edges; // 0 does not have infeasible edges
        public List<int> obstacles;
        public List<int> edges_obstacles;

        //Collision coefficients
        public Double coef_node_obs;
        public Double coef_edge_obs;

        //Extra variables
        public Random random;
        public Double num_random_nodes;
        public bool creation = false;


        //-------------------------------------------------------------------
        public Individual(Random _random, List<int> join_path)
        {
            random = _random;
            path = join_path;
        }
        //--------------------------------------------------------------------------------

        public Individual(Random _random, Double _num_random_nodes, Double _coef_node_obs, Double _coef_edge_obs)
        {
            random = _random;
            num_random_nodes = _num_random_nodes;
            coef_node_obs = _coef_node_obs;
            coef_edge_obs = _coef_edge_obs;
            creation = true;
        }

        //-------------------------------------------------------------------

        public override string ToString()
        {
            return "fit:" + String.Format("{0:0.0000}", fitness) + " cost:" + String.Format("{0:0.0000}", best_cost) + " len:" + path.Count + " obs_n:" + num_infeasible_nodes + " obs_e:" + num_infeasible_edges;
        }

        //--------------------------------------------------------------------------------
        public bool Equals(Individual other)
        {
            return path.SequenceEqual(other.path);
        }

        //---------------------------------------------------------------------
        private int FindBestNode(ref MeshEnvironment env, ref List<int> fragment_route, ref List<int> current_route, int current_node, int node2)
        {

            List<int> neighboors = env.world[current_node].neighboors;
            List<Double> proximities = new List<Double>();

            for (int i = 0; i < neighboors.Count; i++)
            {
                int neigh_node = neighboors[i];
                Double proximity;
                if (!fragment_route.Contains(neigh_node) && !current_route.Contains(neigh_node))
                {
                    proximity = -1;
                    if (creation)
                    {
                        proximity = env.CalculateProximity(current_node, neigh_node, node2);
                    }
                    else if (!env.obstacles.Contains(neigh_node))
                    {
                        proximity = env.CalculateProximity(current_node, neigh_node, node2);
                    }
                }
                else
                {
                    proximity = -1;
                }

                proximities.Add(proximity);
            }

            Double max_proximity = proximities.Max();

            if (max_proximity != -1)
            {
                int index_max1 = proximities.IndexOf(max_proximity);

                return neighboors[index_max1];
            }
            else
            {
                return -1;
            }


        }

        //---------------------------------------------------------------------
        private List<int> ConnectNodes(ref MeshEnvironment env, ref List<int> current_route, int node1, int node2)
        {
            List<int> fragment_route = new List<int>();
            fragment_route.Add(node1);

            int current_node = node1;

            while (current_node != node2)
            {
                int next_node = FindBestNode(ref env, ref fragment_route, ref current_route, current_node, node2);
                if (next_node != -1)
                {
                    fragment_route.Add(next_node);
                    current_node = next_node;
                }
                else
                {
                    break;
                }

            }

            if (current_node != node2)
            {
                fragment_route = new List<int>();
            }

            return fragment_route;

        }
        //-------------------------------------------------------------------\
        private List<int> GetPseudoRoute(ref MeshEnvironment env)
        {
            //Pseudo ruta //Random Process
            List<int> pseudo_route = new List<int>();
            pseudo_route.Add(env.start_node);

            int i = 0;
            int random_idx = -1;
            Double current_proximity = env.CalculateProximity(env.start_node);


            while (i < num_random_nodes && random_idx != env.final_node - 1)
            {
                random_idx = random.Next(pseudo_route[pseudo_route.Count - 1], env.final_node);
                Double next_proximity = env.CalculateProximity(random_idx);
                if (!env.obstacles.Contains(random_idx) && !pseudo_route.Contains(random_idx) && next_proximity < current_proximity)
                {
                    pseudo_route.Add(random_idx);
                    i++;
                    current_proximity = next_proximity;
                }
            }

            pseudo_route.Add(env.final_node);

            return pseudo_route;
        }
        //-------------------------------------------------------------------
        private List<int> FindRoute(ref MeshEnvironment env)
        {
            //Pseudo ruta
            List<int> pseudo_route = GetPseudoRoute(ref env);

            //Variable para almacenar la nueva ruta
            List<int> route = new List<int>();
            route.Add(env.start_node);

            for (int i = 0; i < pseudo_route.Count - 1; i++)
            {
                int node1 = pseudo_route[i];
                int node2 = pseudo_route[i + 1];

                List<int> fragment_route = ConnectNodes(ref env, ref route, node1, node2);

                if (fragment_route.Count != 0)
                {
                    fragment_route.RemoveAt(0);

                    route = route.Concat(fragment_route).ToList();
                }
                else
                {
                    route = new List<int>();
                    break;
                }

            }

            return route;
        }

        //-------------------------------------------------------------------
        public void CreateRandomIndividual(ref MeshEnvironment env)
        {
            List<int> route = FindRoute(ref env);

            while (route.Count == 0)
            {
                route = FindRoute(ref env);
            }

            path = route;
            SetExtraParameters(ref env);
        }

        //--------------------------------------------------------------------------------
        //Configura los parametros extra de acuerdo a los obstaculos presentes
        public void SetExtraParameters(ref MeshEnvironment env)
        {

            List<int> intersection = path.Intersect(env.obstacles).ToList();

            if (intersection.Count == 0)
            {
                is_feasible = true;
                num_infeasible_nodes = 0;
                num_infeasible_edges = 0;
                obstacles = new List<int>();
                edges_obstacles = new List<int>();
            }
            else
            {
                is_feasible = false;
                obstacles = intersection;
                num_infeasible_nodes = intersection.Count;
                edges_obstacles = GetInfeasibleEdges(ref intersection, ref env);
                num_infeasible_edges = edges_obstacles.Count;
            }
        }

        //--------------------------------------------------------------------------------
        public List<int> GetInfeasibleEdges(ref List<int> intersection, ref MeshEnvironment env)
        {
            List<int> edges = new List<int>();

            for (int i = 0; i < path.Count - 1; i++)
            {
                if (intersection.Contains(path[i]) && intersection.Contains(path[i + 1]))
                {
                    Node node1 = new Node(path[i]);
                    Node node2 = new Node(path[i + 1]);
                    Edge new_edge = new Edge(node1, node2);

                    int idx_edge = env.edges.IndexOf(new_edge);

                    edges.Add(idx_edge);

                }
            }

            return edges;
        }

        //-------------------------------------------------------------------------
        //Funcion fitness puede variar de acuerdo al requerimiento del algoritmo
        public void SetFitness(ref MeshEnvironment env)
        {
            //Costo general por la distancia del camino
            Double cost = ExtraTools.GetCost(ref path, ref env);

            //Costo extra por aristas obstaculos
            if (edges_obstacles.Count != 0)
            {
                foreach (var idx_edge in edges_obstacles)
                {
                    cost += env.edges[idx_edge].distance * coef_edge_obs;
                }
            }

            //Costo extra por nodos obstaculos
            cost += num_infeasible_nodes * coef_node_obs;

            fitness = 1 / cost;
            best_cost = cost;
        }

        //--------------------------------------------------------------

        public List<int> JoinGenes(ref List<int> part_1, ref List<int> part_2, ref MeshEnvironment env)
        {
            List<int> final_path = new List<int>(part_1);

            List<int> tmp = new List<int>();

            List<int> fragment = ConnectNodes(ref env, ref tmp, part_1[part_1.Count - 1], part_2[0]);

            if (fragment.Count != 0)
            {
                fragment.RemoveAt(0);
                fragment.RemoveAt(fragment.Count - 1);
                final_path = final_path.Concat(fragment).ToList();
                final_path = final_path.Concat(part_2).ToList();
            }
            else
            {
                final_path = new List<int>();
            }



            return final_path;

        }

        //--------------------------------------------------------------
        //Crossover function
        public Individual Crossover(Individual parent2, ref MeshEnvironment env)
        {
            int piece1 = path.Count / 3;
            int piece2 = parent2.path.Count / 3;

            int cut1 = random.Next(piece1, piece1 * 2 + 1);
            int cut2 = random.Next(piece2, piece2 * 2 + 1);
            //int cut1 = path.Count / 2;
            //int cut2 = parent2.path.Count / 2;

            //Divido los padres. padre 1 obtengo la parte izquierda y del padre 2 la parte derecha
            List<int> part_1 = path.GetRange(0, cut1);
            List<int> part_2 = parent2.path.GetRange(cut2, parent2.path.Count - cut2);

            //Tambien podria tomar aleatoriamente los puntos de corte y hasta encontrar una ruta que
            //no contenga los mismos nodos seguir probando, primero la interseccion

            //Uno las dos partes y reparo la ruta si los nodos no estan directamente conectados
            List<int> join_path;
            if (part_1[part_1.Count - 1] == part_2[0])
            {
                part_2.RemoveAt(0);
                join_path = part_1.Concat(part_2).ToList();
            }
            else
            {
                join_path = JoinGenes(ref part_1, ref part_2, ref env);
            }

            //Create the new individual
            Individual child = new Individual(random, join_path);
            child.SetExtraParameters(ref env);
            child.SetFitness(ref env);

            return child;
        }

        //-------------------------------------------------------------------------------

        //This function takes one node and change it randomly by other node randomly
        public void Mutation(ref MeshEnvironment env, int len_cut, Double mutation_prob)
        {
            Double random_prob = random.NextDouble();

            if (random_prob < mutation_prob)
            {
                //Elijo el nodo que voy a cambiar
                int random_cut1 = random.Next(1, path.Count() - 2 - len_cut);
                int random_cut2 = random.Next(random_cut1 + 1, random_cut1 + 1 + len_cut);

                //Realizo los dos cortes de la ruta

                List<int> fragment1 = path.GetRange(0, random_cut1);
                List<int> fragment2 = path.GetRange(random_cut2, path.Count - random_cut2);

                //Uno las dos partes y reparo la ruta si los nodos no estan directamente conectados
                List<int> join_path;
                if (fragment1[fragment1.Count - 1] == fragment2[0])
                {
                    fragment2.RemoveAt(0);
                    join_path = fragment1.Concat(fragment2).ToList();
                }
                else
                {
                    join_path = JoinGenes(ref fragment1, ref fragment2, ref env);
                }
                path = join_path;
                SetExtraParameters(ref env);
                SetFitness(ref env);
            }

        }

        //--------------------------------------------------------------------------------
        //This functions only works in a feasible path. It takes one random node and checks if
        //its neighboors can have a better cost with other conexion of the commons neighboors
        public void Improve(ref MeshEnvironment env)
        {
            if (is_feasible)
            {
                int random_idx = random.Next(1, path.Count - 1);

                //Get the previous and next node of that obstacle in the path
                int previous_node = path[random_idx - 1];
                int next_node = path[random_idx + 1];

                List<int> neigh_previous = env.world[previous_node].neighboors;
                List<int> neigh_next = env.world[next_node].neighboors;
                List<int> intersection = neigh_previous.Intersect(neigh_next).ToList();
                intersection.Remove(path[random_idx]);

                if (intersection.Count != 0 && !env.obstacles.Contains(intersection[0]))
                {
                    List<int> tmp_path = new List<int>(path);
                    tmp_path[random_idx] = intersection[0];
                    Double tmp_cost = ExtraTools.GetCost(ref tmp_path, ref env);

                    if (tmp_cost < best_cost)
                    {
                        best_cost = tmp_cost;
                        path = new List<int>(tmp_path);
                        SetExtraParameters(ref env);
                        SetFitness(ref env);
                    }

                }

            }
        }
    }

}