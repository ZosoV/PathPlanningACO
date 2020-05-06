using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathPlanningACO.ACO
{
    class RandomWalkv1
    {
        //Parameters of Random Walk
        public int num_random_walks = 20;           //Number of max random walks
        public Double delta_tau = 0.2;              //Initial delta_tau for reinforcement process, 
                                                    //the value of delta_tau changes in execution time
        public int random_movements = 2;            //Factor k the number of best proximities nodes taken into account
        public Double evaporation_factor = 0.25;    //The rho value from the evaporation of random walks


        //Setting for random values
        static int seed = Environment.TickCount;
        static Random random = new System.Random(seed);

        //Variables to store the best route so far and its cost
        public List<int> best_route = new List<int>();
        public Double best_cost = Double.MaxValue;

        //TESTING VARIABLES
        public Double execution_time = 0.0;

        //-------------------------------------------------------------------
        //The value changes in execution time taking into account the best solution found so far.
        private void DeltaTau(ref MeshEnvironment env, ref List<int> route)
        {
            if (best_cost != Double.MaxValue)
            {
                Double current_cost = ExtraTools.GetCost(ref route, ref env);
                delta_tau = best_cost / current_cost;
            }

        }
        //-------------------------------------------------------------------
        public void CheckAndSetBestCost(ref MeshEnvironment env, ref List<int> route)
        {
            Double current_cost = ExtraTools.GetCost(ref route, ref env);

            if (current_cost < best_cost)
            {
                best_route = route;
                best_cost = current_cost;
            }
        }
        //-------------------------------------------------------------------
        public void Evaporation(ref MeshEnvironment env)
        {

            for (int i = 0; i < env.edges.Count; i++)
            {
                env.edges[i].pheromone_amount = env.edges[i].pheromone_amount * (1 - evaporation_factor);
            }
        }
        //-------------------------------------------------------------------
        private void Reinforcement(ref MeshEnvironment env, ref List<int> route)
        {
            for (int i = 0; i < route.Count - 1; i++)
            {
                int node1_idx = route[i];
                int node2_idx = route[i + 1];

                int index = env.world[node1_idx].neighboors.IndexOf(node2_idx);

                int edge_idx = env.world[node1_idx].edges[index];
                DeltaTau(ref env, ref route);
                env.edges[edge_idx].pheromone_amount += delta_tau;

            }
        }

        //---------------------------------------------------------------------
        private int SelectRandomNode(ref MeshEnvironment env, ref List<int> current_route, int current_node)
        {
            int next_node = -1;

            //Get the info the current node: neighboors node, edges, and proximities
            List<int> possible_next_nodes = new List<int>(env.world[current_node].neighboors);

            if (possible_next_nodes.Count <= random_movements && !current_route.Contains(possible_next_nodes[0]))
            {
                int random_idx = random.Next(0, possible_next_nodes.Count);
                next_node = possible_next_nodes[random_idx];
            }
            else
            {
                List<Double> proximities = new List<Double>(env.world[current_node].proximities);

                List<int> node_idxs = new List<int>();

                for (int i = 0; i < random_movements; i++)
                {
                    Double max_proximity = proximities.Max();

                    int index = proximities.IndexOf(max_proximity);
                    proximities[index] = -1;

                    int node_idx = possible_next_nodes[index];

                    if (!current_route.Contains(node_idx))
                    {
                        node_idxs.Add(node_idx);
                    }
                }

                if (node_idxs.Count != 0)
                {
                    int random_idx = random.Next(0, node_idxs.Count);
                    next_node = node_idxs[random_idx];
                }

            }



            return next_node;
        }

        //-------------------------------------------------------------------

        private List<int> FindRoute(ref MeshEnvironment env)
        {
            //Variable to store the new route
            List<int> route = new List<int>();
            route.Add(env.start_node); //Add the initial node

            //Variable to detect if the random walk was found
            bool found_route = false;

            //Init the loop from the initial node
            int current_node = env.start_node;
            int next_node = -1;
            while (!found_route)
            {

                next_node = SelectRandomNode(ref env, ref route, current_node);


                if (next_node != -1)
                {
                    //Add the new node and continue to the next position
                    route.Add(next_node);
                    current_node = next_node;
                }
                else
                {
                    //If there is a stuck condition, the algorithm breaks the loop
                    break;
                }
                //Set the variable, if the random walk is found
                found_route = current_node == env.final_node;
            }

            //If theres is a stuck condition, delete the current nodes of the route
            if (!found_route)
            {
                route = new List<int>();
            }


            return route;
        }

        //-------------------------------------------------------------------

        public void ExecuteRandomWalk(ref MeshEnvironment env)
        {
            int counter = 0;

            //Variable to take the time
            var watch = System.Diagnostics.Stopwatch.StartNew();

            while (counter != num_random_walks)
            {
                List<int> new_route = FindRoute(ref env);
                if (new_route.Count != 0)
                {
                    Reinforcement(ref env, ref new_route);
                    CheckAndSetBestCost(ref env, ref new_route);
                    counter++;
                    Evaporation(ref env);
                }

            }
            //Execution Time
            watch.Stop();
            execution_time = watch.ElapsedMilliseconds;
        }
    }

}
