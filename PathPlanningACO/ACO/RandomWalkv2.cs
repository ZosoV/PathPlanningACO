using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathPlanningACO.ACO
{
    class RandomWalkv2
    {
        public int num_random_walks = 20;                //Number of max random walks
        public Double delta_tau = 0.2;              //Initial delta_tau for reinforcement process
        public Double evaporation_factor = 0.25;    //The rho value from the evaporation of random walks
        public int num_random_nodes = 10;           //Factor h of intermediate random nodes

        //Setting random values
        static int seed = Environment.TickCount;
        static Random random = new System.Random(seed);

        public Double best_cost = Double.MaxValue;

        //TESTING VARIABLES
        public Double execution_time = 0.0;

        //-------------------------------------------------------------------

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
        //This function returns the neighbor node that has the best proximity value
        private int FindBestNode(ref MeshEnvironment env, ref List<int> fragment_route, ref List<int> current_route, int current_node, int node2)
        {

            List<int> neighboors = env.world[current_node].neighboors;
            List<Double> proximities = new List<Double>();

            for (int i = 0; i < neighboors.Count; i++)
            {
                int neigh_node = neighboors[i];
                Double proximity;
                if (!fragment_route.Contains(neigh_node) && !current_route.Contains(neigh_node) && !env.obstacles.Contains(neigh_node))
                {
                    proximity = env.CalculateProximity(current_node, neigh_node, node2);
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
        //This function takes two nodes, and tries to connect then following the best proximity in each step
        private List<int> ConnectNodes(ref MeshEnvironment env, ref List<int> current_route, int node1, int node2)
        {
            //Init the fragment route with the node1
            List<int> fragment_route = new List<int>();
            fragment_route.Add(node1);

            //Init the loop from the node1 to the node2
            int current_node = node1;
            while (current_node != node2)
            {
                //In each step I select the node that has the best proximity value
                int next_node = FindBestNode(ref env, ref fragment_route, ref current_route, current_node, node2);
                if (next_node != -1)
                {
                    //If the node is not null add to the fragment list
                    fragment_route.Add(next_node);
                    current_node = next_node;
                }
                else
                {
                    //If there is an stuck condition, the algorithm breaks the loop
                    break;
                }

            }

            //If there is an stuck condition, return a empty fragment
            if (current_node != node2)
            {
                fragment_route = new List<int>();
            }

            return fragment_route;

        }
        //-------------------------------------------------------------------\
        private List<int> GetPseudoRoute(ref MeshEnvironment env)
        {
            //Pseudo route //Random Process
            List<int> pseudo_route = new List<int>();
            pseudo_route.Add(env.start_node);

            int i = 0;
            int random_idx = -1;
            Double current_proximity = env.CalculateProximity(env.start_node);

            //Init the loop from the initial node and chose a node randomly
            while (i < num_random_nodes && random_idx != env.final_node - 1)
            {
                //In each step, I verify that the new random node is farther than the previous one, checking the proximity value of that node. 
                random_idx = random.Next(pseudo_route[pseudo_route.Count - 1], env.final_node);
                Double next_proximity = env.CalculateProximity(random_idx);
                if (!env.obstacles.Contains(random_idx) && !pseudo_route.Contains(random_idx) && next_proximity < current_proximity)
                {
                    //Only add the a new random node that it is further that the previous one
                    pseudo_route.Add(random_idx);
                    i++;
                    current_proximity = next_proximity;
                }
            }

            //At the end, add the final node
            pseudo_route.Add(env.final_node);

            return pseudo_route;
        }

        //-------------------------------------------------------------------

        private List<int> FindRoute(ref MeshEnvironment env)
        {
            //Pseudo route or unlinked route
            List<int> pseudo_route = GetPseudoRoute(ref env);

            //Variable to store the new route (completed route)
            List<int> route = new List<int>();
            route.Add(env.start_node);

            for (int i = 0; i < pseudo_route.Count - 1; i++)
            {
                //Take two nodes of the unlinked route in each iteration
                int node1 = pseudo_route[i];
                int node2 = pseudo_route[i + 1];

                //Try to connect both nodes and get that fragment of route
                List<int> fragment_route = ConnectNodes(ref env, ref route, node1, node2);

                //If the fragment is not empty
                if (fragment_route.Count != 0)
                {
                    //Concat the fragment to the current route
                    fragment_route.RemoveAt(0);
                    route = route.Concat(fragment_route).ToList();
                }
                else
                {
                    //else destroy the route and return an empty list
                    route = new List<int>();
                    break;
                }

            }

            return route;
        }

        //-------------------------------------------------------------------
        public void ExecuteRandomWalk(ref MeshEnvironment env)
        {
            int counter = 0;

            //Variable to take the execution time
            var watch = System.Diagnostics.Stopwatch.StartNew();

            while (counter != num_random_walks)
            {
                List<int> new_route = FindRoute(ref env);

                if (new_route.Count != 0)
                {
                    Reinforcement(ref env, ref new_route);
                    CheckAndSetBestCost(ref env, ref new_route);
                    Evaporation(ref env);
                    counter++;
                }

            }

            //Execution Time
            watch.Stop();
            execution_time = watch.ElapsedMilliseconds;

        }
    }

}
