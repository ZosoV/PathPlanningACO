using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.Text;

namespace PathPlanningACO.ACO
{
    class Antv0
    {
        public int id;
        public List<int> current_route;
        public Double current_cost;
        public int counter_route;
        public Double alpha;
        public Double beta;
        public Random random;
        public Double tau_0;
        public bool use_random_walk;

        //-------------------------------------------------------------------
        public Antv0(int _id, Double _alpha, Double _beta, Double _tau_0, Random _random, bool _use_random_walk)
        {
            id = _id;
            current_route = new List<int>();
            counter_route = 0;
            alpha = _alpha;
            beta = _beta;
            tau_0 = _tau_0;
            random = _random;
            use_random_walk = _use_random_walk;
        }

        //-------------------------------------------------------------------
        public override string ToString()
        {
            return "id: " + id + "  route counter: " + counter_route + " current length: " + current_route.Count;

        }

        //-------------------------------------------------------------------
        public Double ComputeCoefficient(Double pheromone, Double distance, Double proximity)
        {
            //If the algorithm is set with only ACOv0, we use these conditions
            if (!use_random_walk)
            {
                if (counter_route < 3  || pheromone == tau_0  )
                {
                    return Math.Pow(proximity, alpha) * Math.Pow(1 / distance, beta);
                }
                else
                {
                    return Math.Pow(pheromone, alpha) * Math.Pow(proximity, beta);
                }
            }
            //If the algorithm is set with the random walks, we use this condition
            else
            {
                return Math.Pow(pheromone, alpha) * Math.Pow(proximity, beta) * Math.Pow(1 / distance, beta);
            }

        }

        //-------------------------------------------------------------------
        public int SelectNextNode(ref MeshEnvironment env, int current_node)
        {

            //Get the info the current node: neighboors node, edges, and proximities
            List<int> possible_next_nodes = env.world[current_node].neighboors;

            //Variable to store the next node
            int next_node = -1;
            int node_idx_only = possible_next_nodes[0];

            //Implementar esto para reducir tiempo, en los resultados OJAZO
            if (possible_next_nodes.Count == 1 && env.world[node_idx_only].GetVisited(id) == false && !env.obstacles.Contains(node_idx_only))
            {
                next_node = node_idx_only;
                env.world[current_node].visited_by = id;

            }
            else
            {
                List<int> possible_next_edges = env.world[current_node].edges;
                List<Double> proximities = env.world[current_node].proximities;


                //Variable to store the acumulative coefficients
                List<Double> list_acu_coeff = new List<Double>();
                List<int> indexes = new List<int>();
                Double accu_coefficient = 0;

                for (int i = 0; i < possible_next_nodes.Count; i++)
                {
                    int node_idx = possible_next_nodes[i];
                    int edge_idx = possible_next_edges[i];

                    //Select the next node only if the current ant not visited the next node.
                    if (env.world[node_idx].GetVisited(id) == false && !env.obstacles.Contains(node_idx))
                    {
                        //Heuristic criteria and pheromones
                        Double pheromone = env.edges[edge_idx].pheromone_amount;
                        Double proximity = proximities[i];
                        Double distance = env.edges[edge_idx].distance;

                        accu_coefficient += ComputeCoefficient(pheromone, distance, proximity);
                        list_acu_coeff.Add(accu_coefficient);
                        indexes.Add(i);

                    }

                }

                //SELECT FOLLOWING A DOOR CRITERIA
                if (accu_coefficient != 0 && !Double.IsInfinity(accu_coefficient))
                {
                    Double random_number = random.NextDouble() * accu_coefficient;

                    for (int i = 0; i < list_acu_coeff.Count; i++)
                    {
                        int node_idx = possible_next_nodes[indexes[i]];

                        if (random_number < list_acu_coeff[i])
                        {
                            next_node = node_idx;
                            env.world[current_node].visited_by = id;

                            break;
                        }
                    }
                }
                else if (Double.IsInfinity(accu_coefficient))
                {
                    next_node = env.final_node;
                }


            }


            return next_node;
        }

        //-------------------------------------------------------------------
        public List<int> FindRoute(ref MeshEnvironment env)
        {
            //List to store the new route
            List<int> route = new List<int>();
            route.Add(env.start_node); //Include the initial node

            //Variable to detect if the ant finds a route
            bool found_route = false;

            //Init the loop from the initial node
            int current_node = env.start_node;
            while (!found_route)
            {

                int next_node = SelectNextNode(ref env, current_node);
                if (next_node != -1)
                {
                    //Add the new node and change the current node
                    route.Add(next_node);
                    current_node = next_node;
                }
                else
                {
                    //If there is a stuck condition break the loop
                    break;
                }

                //Configuro mi variable si he encontrado el objetivo
                found_route = current_node == env.final_node;
            }

            //If the ant found a route
            if (found_route)
            {
                //Set the current route, the current cost and the current counter
                current_route = route;
                current_cost = ExtraTools.GetCost(ref current_route, ref env);
                counter_route++;
            }
            else
            {
                route = new List<int>();
            }


            return route;
        }

    }

}
