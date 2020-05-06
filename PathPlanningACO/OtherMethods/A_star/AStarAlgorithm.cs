using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathPlanningACO.OtherMethods.A_star
{
    class AStarAlgorithm
    {
        public Tag[] node_tags;
        public List<int> final_best_path;
        public Double final_best_cost;
        public Double execution_time;

        //-------------------------------------------------------------------------------

        public void InitNodeTags(int size)
        {
            node_tags = new Tag[size];
            for (int i = 0; i < size; i++)
            {
                Tag new_tag = new Tag(i, false, Double.MaxValue, Double.MaxValue, null);
                node_tags[i] = new_tag;

            }
        }
        //-------------------------------------------------------------------------------

        public Double MeasureHeuristic(int current_node, int node1, ref MeshEnvironment env)
        {
            Double heuristic_value;
            if (current_node == env.start_node && node1 == env.start_node)
            {
                heuristic_value = env.world[node1].distance_to_final;

            }
            else
            {
                //heuristic_value = 1 / env.CalculateProximity(current_node, node1);
                int idx = env.world[current_node].neighboors.IndexOf(node1);
                heuristic_value = 1 / env.world[current_node].proximities[idx];
            }

            return heuristic_value;
        }


        //-------------------------------------------------------------------------------
        // Get the posible nodes where the ant can move
        public List<int> GetPosibleNodes(int current_position, ref MeshEnvironment env)
        {

            //Obtengo los posibles nodos a los cuales me puedo mover             //Primer filtro solo los que vertices conectados
            List<int> list_posible_nodes = new List<int>(env.world[current_position].neighboors);


            //Filtrar obstaculos
            list_posible_nodes = list_posible_nodes.Except(env.obstacles).ToList();

            //Filtrar nodos que ya se han visitado
            //posible_nodes = posible_nodes.Except(posible_nodes.Intersect(current_path)).ToList();
            return list_posible_nodes;
        }

        //-------------------------------------------------------------------------------
        //Funcion para obetner el camino una vez ejectuado el algoritmo A-star
        public List<int> GetPath(int final_position, int initial_position)
        {
            int current_position = final_position;
            List<int> path = new List<int>();
            path.Add(current_position);

            while (current_position != initial_position)
            {
                int parent = node_tags[current_position].parent.Value;
                path.Add(parent);
                current_position = parent;
            }

            return path;
        }

        //-------------------------------------------------------------------------------
        //Funcion de coste del camino actual
        public static Double GetCost(List<int> path, Double[,] distances_matrix)
        {
            Double cost = 0;
            for (int i = 0; i < path.Count - 1; i++)
            {
                cost += distances_matrix[path.ElementAt(i), path.ElementAt(i + 1)];
            }
            return cost;
        }

        //-------------------------------------------------------------------------------

        public void Execute(ref MeshEnvironment env)
        {
            //Fill distances to objective
            env.FillDistancesToFinal();

            //Init the node tags list;
            int size = env.size * env.size;
            InitNodeTags(size);

            //Get the distance between node. Adjacency Matrix
            env.FillDistanceMatrix();
            Double[,] distances_matrix = env.distances;


            //Get initial and final nodes
            int initial_position = env.start_node;
            int final_position = env.final_node;

            //Get world of the env
            List<Node> world = env.world;

            //Init the values of the initial node and its tag
            node_tags[initial_position].local_goal = 0.0;
            node_tags[initial_position].global_goal = MeasureHeuristic(initial_position, initial_position, ref env);

            //Fijar la posicion actual 
            Tag current_position = node_tags[initial_position];

            //Nodos que seran testeados
            List<Tag> tested_nodes_list = new List<Tag>();


            tested_nodes_list.Add(node_tags[initial_position]);

            //Time variable
            var watch = System.Diagnostics.Stopwatch.StartNew();


            while (tested_nodes_list.Count != 0) // && current_position.node_id != env.final_node)
            {
                //Ordenar la lista de acuerdo al objetivo global en orden ascendente
                tested_nodes_list = tested_nodes_list.OrderBy(tag => tag.global_goal).ToList();

                //Remover de la lista si el nodo analizado ya ha sido visitado
                while (tested_nodes_list.Count != 0 && tested_nodes_list[0].visited)
                {
                    tested_nodes_list.RemoveAt(0);
                }

                //Si la lista es vacia salimos del bucle
                if (tested_nodes_list.Count == 0)
                {
                    break;
                }

                //Inicializamos nuestra posicion actual 
                current_position = tested_nodes_list[0];
                current_position.visited = true;
                tested_nodes_list[0] = current_position;
                node_tags[current_position.node_id] = current_position;


                //Obtengo los posibles nodos a los cuales me puedo mover
                List<int> list_posible_nodes = GetPosibleNodes(current_position.node_id, ref env);

                //Itero por los posibles nodos
                foreach (var neighbour_id in list_posible_nodes)
                {

                    int current_id = current_position.node_id;

                    Double possible_lower_goal = current_position.local_goal + distances_matrix[current_id, neighbour_id];

                    if (possible_lower_goal < node_tags[neighbour_id].local_goal && !node_tags[neighbour_id].visited)
                    {
                        Tag new_tag = node_tags[neighbour_id];
                        new_tag.parent = current_id;
                        new_tag.local_goal = possible_lower_goal;
                        new_tag.global_goal = new_tag.local_goal + MeasureHeuristic(current_id, neighbour_id, ref env);
                        node_tags[neighbour_id] = new_tag;

                        if (tested_nodes_list.Contains(new_tag))
                        {
                            int idx = tested_nodes_list.IndexOf(new_tag);
                            tested_nodes_list[idx] = new_tag;
                        }

                    }

                    //Si los nodos analizados aun no han sido marcados como visitados
                    //Los añadimos a lista de testeo
                    if (!node_tags[neighbour_id].visited && !tested_nodes_list.Contains(node_tags[neighbour_id]))
                    {
                        tested_nodes_list.Add(node_tags[neighbour_id]);
                    }

                }
            }


            List<int> optimal_path = GetPath(final_position, initial_position);
            Double optimal_cost = ExtraTools.GetCost(ref optimal_path, ref env);

            final_best_path = optimal_path;
            final_best_path.Reverse();
            final_best_cost = optimal_cost;

            //Execution Time
            watch.Stop();
            execution_time = watch.ElapsedMilliseconds;

        }
    }

}
