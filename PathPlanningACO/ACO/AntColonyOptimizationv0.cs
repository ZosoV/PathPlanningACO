using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathPlanningACO.ACO
{
    class AntColonyOptimizationv0
    {
        //PARAMETERS
        public int max_ants = 15;                       //Max number of pheromones
        public Double alpha = 3;                        //Alfa pheromones
        public Double beta = 2;                         //Beta distance or proximity
        public Double tau_0 = 0;                        //Initial amount of pheromones
        public Double evaporation_factor = 0.50;        //Rho value. Speed of Evaporation
        public Double delta_tau = 0.2;                  //Reinforcement Value
        public Double percentage_convergence = 0.70;    //Convergence

        //Variables of RandomWalk;
        public bool random_walk = false;

        //Setting for random values
        static int seed = Environment.TickCount;
        static Random random = new System.Random(seed);

        //Store the ants variable
        public List<Antv0> colony;

        //Variables to store the best route so far and its cost
        static List<int> best_route = new List<int>();
        public Double best_cost = Double.MaxValue;



        //TESTING VARIABLES
        public int episode_counter = 0;
        public Double execution_time = 0.0;
        public int stuck_roads = 0;

        //----------------------------------------------------
        public AntColonyOptimizationv0()
        {
        }
        //----------------------------------------------------
        public AntColonyOptimizationv0(bool _random_walk)
        {
            random_walk = _random_walk;
        }
        //----------------------------------------------------

        //Function to create a n number of ants

        public List<Antv0> CreateColony()
        {
            List<Antv0> colony = new List<Antv0>();

            for (int i = 0; i < max_ants; i++)
            {
                Antv0 ant = new Antv0(i, alpha, beta, tau_0, random, random_walk);
                colony.Add(ant);
            }

            return colony;
        }

        //-------------------------------------------------------------------
        //Evaporation Function

        public void Evaporation(ref MeshEnvironment env)
        {

            for (int i = 0; i < env.edges.Count; i++)
            {
                env.edges[i].pheromone_amount = env.edges[i].pheromone_amount * (1 - evaporation_factor);
            }
        }

        //-------------------------------------------------------------------
        public void DeltaTau(ref MeshEnvironment env, ref List<int> route)
        {
            if (best_cost != Double.MaxValue)
            {
                Double current_cost = ExtraTools.GetCost(ref route, ref env);
                delta_tau = best_cost / current_cost;
            }

        }

        //-------------------------------------------------------------------

        public void Reinforcement(ref MeshEnvironment env, ref List<int> route)
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
        public bool CheckConvergence(ref List<Antv0> colony, ref MeshEnvironment env)
        {
            int num_convergence = (int)Math.Round(max_ants * percentage_convergence);

            Double best_cost_ant = Double.MaxValue;
            int best_ant = -1;
            for (int k = 0; k < colony.Count; k++)
            {
                Double current_cost = colony[k].current_cost;
                if (current_cost < best_cost_ant)
                {
                    best_cost_ant = current_cost;
                    best_ant = k;
                }
            }


            List<int> best_route_ant = colony[best_ant].current_route;

            int i = 0, j = 0;
            while (i < num_convergence && j < max_ants)
            {
                if (colony[j].current_route.SequenceEqual(best_route_ant))
                {
                    i++;
                }
                j++;
            }

            if (best_route_ant.Count != 0)
            {
                if (i == num_convergence)
                {
                    best_cost = best_cost_ant;
                    best_route = best_route_ant;
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }



        }

        //-------------------------------------------------------------------
        public void ExecuteACO(ref MeshEnvironment env)
        {
            //Create a list of ants
            colony = CreateColony();

            if (!random_walk)
            {
                //Init pheromone minimum amount
                env.InitPheromones(tau_0);
            }
            //Variables  to control the evaporation
            bool almost_one_route = false;
            bool several_stucks = false;

            //Variable to control the convergence
            bool converge = false;

            //Variable to take the execution time
            var watch = System.Diagnostics.Stopwatch.StartNew();

            while (!converge)
            {
                foreach (var ant in colony)
                {
                    //The current ant tries to find a route
                    List<int> route = ant.FindRoute(ref env);

                    //If the ant finds a route
                    if (route.Count != 0)
                    {

                        //Reinforce the route
                        Reinforcement(ref env, ref route);

                        //Set the flag to the evaporation in the current episode
                        almost_one_route = true;

                        //Check and set if the best cost changed
                        CheckAndSetBestCost(ref env, ref route);

                    }
                    else
                    {
                        //Control Testing Variables
                        stuck_roads++;

                        //Several stucks
                        if( stuck_roads % 10 == 0)
                        {
                            several_stucks = true;
                        }
                    }
                }

                env.ResetNodes();

                //If at least one ant found a route the evaporation step occurs
                if (almost_one_route || several_stucks)
                {
                    Evaporation(ref env);
                    almost_one_route = false;
                    several_stucks = false;
                }

                //If a percentage of the ants follow the same path, the convergence variable will be true
                converge = CheckConvergence(ref colony, ref env);

                //Control Testing Variables
                //Episodes
                episode_counter++;
            }

            //Control Testing Variables
            //Execution Time
            watch.Stop();
            execution_time += watch.ElapsedMilliseconds;

        }
    }

}
