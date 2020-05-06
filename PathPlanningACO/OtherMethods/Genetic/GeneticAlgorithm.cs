using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathPlanningACO.OtherMethods.Genetic
{
    class GeneticAlgorithm
    {
        //Cantidad de individuos en este algoritmo.
        public int num_individuals = 20;

        //Number of max iterations
        public int num_iter = 0;

        //List of the population
        public List<Individual> population;

        //Longitud maxima de los individuos
        public int num_random_nodes = 10;

        //Collision coefficients
        public Double coef_edge_obs = 0.7;
        public Double coef_node_obs = 0.8;

        //Parametros de control de la evolucion
        public Double survival_prob = 0.4;
        public Double mutation_prob = 0.8;
        public Double percentage_convergence = 0.5;

        //Genetic Operator parameters
        public int len_cut_mutation = 4;


        //Ajustes de valores random
        public static int seed; //= Environment.TickCount;
        //static int seed = 1335421; //Para detectar el error

        public Random random; //= new System.Random(seed);

        public List<int> final_best_path;
        public Double final_best_cost;

        //TESTING VARIABLES
        public Double execution_time = 0.0;

        public GeneticAlgorithm()
        {
            seed = Environment.TickCount;
            random = new System.Random(seed);

        }

        //---------------------------------------------------------------------------
        //Function for creating the initial random population
        public void CreateInitialPopulation(ref MeshEnvironment env)
        {
            population = new List<Individual>();
            for (int i = 0; i < num_individuals; i++)
            {
                Individual ind = new Individual(random, num_random_nodes, coef_node_obs, coef_edge_obs);
                ind.CreateRandomIndividual(ref env);
                while (population.Contains(ind))
                {
                    ind = new Individual(random, num_random_nodes, coef_node_obs, coef_edge_obs);
                    ind.CreateRandomIndividual(ref env);
                }

                population.Add(ind);
            }

        }

        //--------------------------------------------------------------
        //Funcion para calcular el fitness de todos los individuos de una generación
        public void MeasureFitness(ref MeshEnvironment env)
        {
            foreach (var individual in population)
            {
                individual.SetFitness(ref env);
            }
        }

        //--------------------------------------------------------------
        public bool CheckPopulation(ref MeshEnvironment env)
        {
            int num_convergence = (int)Math.Round(num_individuals * percentage_convergence);

            Double best_cost_individual = Double.MaxValue;
            int best_individual = -1;
            for (int k = 0; k < population.Count; k++)
            {
                Double current_cost = population[k].best_cost;
                if (current_cost < best_cost_individual)
                {
                    best_cost_individual = current_cost;
                    best_individual = k;
                }
            }


            List<int> best_route_individual = population[best_individual].path;

            int i = 0, j = 0;
            while (i < num_convergence && j < num_individuals)
            {
                //if (population[j].path.SequenceEqual(best_route_individual))
                if (Math.Round(population[j].best_cost, 4) == Math.Round(best_cost_individual, 4))
                {
                    i++;
                }
                j++;
            }


            bool converge = false;

            if (best_route_individual.Count != 0)
            {
                if (i == num_convergence)
                {
                    final_best_path = population[0].path;
                    final_best_cost = ExtraTools.GetCost(ref final_best_path, ref env);
                    converge = true;
                }
            }

            //Probemos con el segundo mejor
            if (converge == false)
            {
                best_cost_individual = Double.MaxValue;
                best_individual = -1;
                for (int k = 1; k < population.Count; k++)
                {
                    Double current_cost = population[k].best_cost;
                    if (current_cost < best_cost_individual)
                    {
                        best_cost_individual = current_cost;
                        best_individual = k;
                    }
                }


                best_route_individual = population[best_individual].path;

                i = 0;
                j = 0;
                while (i < (num_convergence - 1) && j < num_individuals)
                {
                    if (Math.Round(population[j].best_cost, 4) == Math.Round(best_cost_individual, 4))
                    {
                        i++;
                    }
                    j++;
                }

                if (best_route_individual.Count != 0)
                {
                    if (i == num_convergence - 1)
                    {
                        final_best_path = population[0].path;
                        final_best_cost = ExtraTools.GetCost(ref final_best_path, ref env);
                        converge = true;
                    }
                }
            }

            return converge;

        }

        //--------------------------------------------------------------

        public List<Double> GetProbabilities(int num_survivals)
        {
            List<Double> probabilities = new List<Double>();

            Double piece = (Double)1 / (Double)num_survivals;

            for (int i = num_survivals; i > 0; i--)
            {
                probabilities.Add(piece * i);
            }

            return probabilities;
        }


        //--------------------------------------------------------------
        public void Execute(ref MeshEnvironment env)
        {
            //Console.WriteLine(seed);

            //Time Variable
            var watch = System.Diagnostics.Stopwatch.StartNew();

            CreateInitialPopulation(ref env);

            //Get the fitness of the current generation
            MeasureFitness(ref env);

            //Order the population in descending order according to the fitness
            population = population.OrderByDescending(ind => ind.fitness).ToList();

            bool converge = false;

            List<Double> probabilities = GetProbabilities((int)Math.Round(num_individuals * survival_prob));

            //for (int iter = 0; iter < num_iter; iter++)
            //while (current_accuracy < 95.0)
            while (!converge)
            {


                //Selection method //Sobreviviente conservo algunos individuos de la generacion anterior
                List<Individual> survivals = population.GetRange(0, (int)Math.Round(num_individuals * survival_prob));

                int num_survivals = survivals.Count;
                int upper_bound = num_individuals - num_survivals;

                for (int i = 0; i < upper_bound; i++)
                {
                    //CROSSOVER
                    //select two random parents //if parents are equal change
                    Double random_prob = random.NextDouble();

                    int idx1 = random.Next(0, num_survivals);

                    while (probabilities[idx1] < random_prob)
                    {
                        idx1 = random.Next(0, num_survivals);
                    }

                    random_prob = random.NextDouble();

                    int idx2 = random.Next(0, num_survivals);

                    while (probabilities[idx2] < random_prob)
                    {
                        idx2 = random.Next(0, num_survivals);
                    }

                    Individual parent1 = survivals[idx1];
                    Individual parent2 = survivals[idx2];

                    while (parent1 == parent2)
                    {
                        random_prob = random.NextDouble();

                        idx2 = random.Next(0, num_survivals);

                        while (probabilities[idx2] < random_prob)
                        {
                            idx2 = random.Next(0, num_survivals);
                        }
                        parent2 = survivals[idx2];
                    }

                    Individual child = parent1.Crossover(parent2, ref env);

                    //////------------------------------------------------
                    //MUTATION
                    if (child.path.Count != 0)
                    {
                        child.Mutation(ref env, len_cut_mutation, mutation_prob);

                    }

                    ////------------------------------------------------
                    ////NODE REPAIR
                    //child.NodeRepair(random, env);
                    //new_ind.SetExtraParameters(env);

                    ////------------------------------------------------
                    ////LINE REPAIR
                    //new_ind.LineRepair(random, env);
                    //new_ind.SetExtraParameters(env);

                    ////------------------------------------------------
                    ////DELETION
                    //new_ind.Deletion(random, env);
                    //new_ind.SetExtraParameters(env);

                    ////------------------------------------------------
                    ////IMPROVE
                    ///

                    if (child.path.Count != 0)
                    {
                        child.Improve(ref env);
                        survivals.Add(child);
                    }

                    if (child.path.Count == 0)
                    {
                        i--;
                    }

                }

                population = survivals;

                //Get the fitness of the current generation
                MeasureFitness(ref env);

                //Order the population in descending order according to the fitness
                population = population.OrderByDescending(ind => ind.fitness).ToList();

                converge = CheckPopulation(ref env);

                //current_accuracy = (optimal_cost * 100) / population[0].best_cost;

                num_iter++;
            }


            //Execution Time
            watch.Stop();
            execution_time += watch.ElapsedMilliseconds;
        }

    }

}
