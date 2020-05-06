using PathPlanningACO.ACO;
using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.OtherMethods.Dijkstra;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace PathPlanningACO.Testing
{
    class TestRandomWalksACO
    {
        public static int num_test = 30;
        //---------------------------------------------------------------------
        public static void StoreVariables_RW1(string mesh_type, ref MeshEnvironment env, ref RandomWalkv1 rw1)
        {
            string separator = " ";
            string line = mesh_type + separator;
            line += rw1.num_random_walks + separator;
            line += rw1.random_movements + separator;
            line += rw1.evaporation_factor + separator;
            line += rw1.best_cost + separator;
            line += rw1.execution_time;

            string path = @"data_sets/results/acov0_rw1/" + mesh_type + "/";
            string variables_file = "info_rw1" + "_" + mesh_type + "_" + env.size + "x" + env.size + ".txt";

            using (StreamWriter file =
             File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }

            string path2 = @"data_sets/results/acov0_rw1/";
            string variables_file2 = "info_rw1" + "_" + mesh_type + ".txt";

            using (StreamWriter file =
                File.AppendText(path2 + variables_file2))
            {
                file.WriteLine(line);
            }
        }
        //---------------------------------------------------------------------
        public static void StoreVariable_RW1_and_ACOv0(string mesh_type, ref MeshEnvironment env, ref AntColonyOptimizationv0 aco, ref RandomWalkv1 rw1)
        {
            env.FillDistanceMatrix();
            DijkstraJustCost dijkstra = new DijkstraJustCost();
            Double best_cost = dijkstra.DijkstraAlgo(graph: env.distances, source: env.start_node, verticesCount: env.final_node + 1);


            Double percentage_learning = MeasureFunctions.CalculatePercentageLearning(ref env);
            string separator = " ";
            string line = mesh_type + separator;
            line += env.world.Count + separator;
            line += env.edges.Count + separator;
            line += best_cost + separator;
            line += aco.max_ants + separator;
            line += aco.alpha + separator;
            line += aco.beta + separator;
            line += aco.tau_0 + separator;
            line += aco.evaporation_factor + separator;
            line += aco.percentage_convergence + separator;
            line += aco.episode_counter + separator;
            line += aco.execution_time + separator;
            line += aco.stuck_roads + separator;
            line += MeasureFunctions.GetVisitedNodes(ref env) + separator;
            line += MeasureFunctions.GetVisitedEdges(ref env) + separator;
            line += Math.Round(aco.best_cost, 2) + separator;
            line += percentage_learning + separator;
            line += rw1.num_random_walks + separator;
            line += rw1.random_movements + separator;
            line += rw1.evaporation_factor + separator;
            line += rw1.best_cost + separator;
            line += rw1.execution_time;

            string path = @"data_sets/results/acov0_rw1/" + mesh_type + "/";
            string variables_file = "variablesarw1" + "_" + mesh_type + "_" + env.size + "x" + env.size + ".txt";

            using (StreamWriter file =
             File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }

            string path2 = @"data_sets/results/acov0_rw1/";
            string variables_file2 = "variablesrw1" + "_" + mesh_type + ".txt";

            using (StreamWriter file =
                File.AppendText(path2 + variables_file2))
            {
                file.WriteLine(line);
            }
        }
        //---------------------------------------------------------------------
        public static void TestRandomWalkv1(string mesh_type)
        {

            int[] sizes_mesh = new int[21] { 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50 };
            int[] final_positions = new int[21] { 99, 143, 195, 255, 323, 399, 483, 575, 675, 783, 899, 1023, 1155, 1295, 1443, 1599, 1763, 1935, 2115, 2303, 2499 };
            for (int i = 0; i < sizes_mesh.Length; i++)
            {
                string size = "_" + sizes_mesh[i] + "x" + sizes_mesh[i] + ".txt";
                string file_name = mesh_type + size;
                Console.WriteLine("Init Execution -> " + file_name);

                //Create the mesh environment
                MeshEnvironment env = new MeshEnvironment(_start: 0, _final: final_positions[i], _file_name: file_name, sizes_mesh[i]);
                env.InitEnviroment( type_mesh: mesh_type);
                //-------------------------------------------------------------------

                for (int j = 0; j < num_test; j++)
                {
                    RandomWalkv1 rw1 = new RandomWalkv1();
                    rw1.ExecuteRandomWalk(ref env);

                    AntColonyOptimizationv0 acov0 = new AntColonyOptimizationv0(_random_walk: true);
                    acov0.ExecuteACO(ref env);

                    //Store variables in a txt
                    Console.WriteLine("Execution {0} of " + file_name, j);
                    StoreVariables_RW1(mesh_type, ref env, ref rw1);
                    StoreVariable_RW1_and_ACOv0(mesh_type, ref env, ref acov0, ref rw1);
                    env.InitPheromones(0);

                }

                Console.WriteLine("End Execution -> " + file_name);
                Console.WriteLine("---------------------------------------------");

            }



        }
        //---------------------------------------------------------------------
        public static void StoreVariables_RW2(string mesh_type, ref MeshEnvironment env, ref RandomWalkv2 rw)
        {
            string separator = " ";
            string line = mesh_type + separator;
            line += rw.num_random_walks + separator;
            line += rw.evaporation_factor + separator;
            line += rw.best_cost + separator;
            line += rw.execution_time + separator;
            line += rw.num_random_nodes;

            string path = @"data_sets/results/acov0_rw2/" + mesh_type + "/";
            string variables_file = "info_rw2" + "_" + mesh_type + "_" + env.size + "x" + env.size + ".txt";

            using (StreamWriter file =
             File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }

            string path2 = @"data_sets/results/acov0_rw2/";
            string variables_file2 = "info_rw2" + "_" + mesh_type + ".txt";

            using (StreamWriter file =
                File.AppendText(path2 + variables_file2))
            {
                file.WriteLine(line);
            }
        }
        //---------------------------------------------------------------------
        public static void StoreVariable_RW2_and_ACOv0(string mesh_type, ref MeshEnvironment env, ref AntColonyOptimizationv0 aco, ref RandomWalkv2 rw)
        {
            env.FillDistanceMatrix();
            DijkstraJustCost dijkstra = new DijkstraJustCost();
            Double best_cost = dijkstra.DijkstraAlgo(graph: env.distances, source: env.start_node, verticesCount: env.final_node + 1);


            Double percentage_learning = MeasureFunctions.CalculatePercentageLearning(ref env);
            string separator = " ";
            string line = mesh_type + separator;
            line += env.world.Count + separator;
            line += env.edges.Count + separator;
            line += best_cost + separator;
            line += aco.max_ants + separator;
            line += aco.alpha + separator;
            line += aco.beta + separator;
            line += aco.tau_0 + separator;
            line += aco.evaporation_factor + separator;
            line += aco.percentage_convergence + separator;
            line += aco.episode_counter + separator;
            line += aco.execution_time + separator;
            line += aco.stuck_roads + separator;
            line += MeasureFunctions.GetVisitedNodes(ref env) + separator;
            line += MeasureFunctions.GetVisitedEdges(ref env) + separator;
            line += Math.Round(aco.best_cost, 2) + separator;
            line += percentage_learning + separator;
            line += rw.num_random_walks + separator;
            line += rw.evaporation_factor + separator;
            line += rw.best_cost + separator;
            line += rw.execution_time + separator;
            line += rw.num_random_nodes;

            string path = @"data_sets/results/acov0_rw2/" + mesh_type + "/";
            string variables_file = "variablesrw2" + "_" + mesh_type + "_" + env.size + "x" + env.size + ".txt";

            using (StreamWriter file =
             File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }

            string path2 = @"data_sets/results/acov0_rw2/";
            string variables_file2 = "variablesrw2" + "_" + mesh_type + ".txt";

            using (StreamWriter file =
                File.AppendText(path2 + variables_file2))
            {
                file.WriteLine(line);
            }
        }
        //---------------------------------------------------------------------
        public static void TestRandomWalkv2(string mesh_type)
        {


            int[] sizes_mesh = new int[21] { 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50 };
            int[] final_positions = new int[21] { 99, 143, 195, 255, 323, 399, 483, 575, 675, 783, 899, 1023, 1155, 1295, 1443, 1599, 1763, 1935, 2115, 2303, 2499 };
            for (int i = 0; i < sizes_mesh.Length; i++)
            {
                string size = "_" + sizes_mesh[i] + "x" + sizes_mesh[i] + ".txt";
                string file_name = mesh_type + size;

                //Create the mesh environment
                MeshEnvironment env = new MeshEnvironment(_start: 0, _final: final_positions[i], _file_name: file_name, sizes_mesh[i]);
                env.InitEnviroment( type_mesh: mesh_type);

                Console.WriteLine("Init Execution -> " + file_name);

                //-------------------------------------------------------------------

                for (int j = 0; j < num_test; j++)
                {
                    RandomWalkv2 rw2 = new RandomWalkv2();
                    rw2.ExecuteRandomWalk(ref env);

                    AntColonyOptimizationv0 acov0 = new AntColonyOptimizationv0(_random_walk: true);
                    acov0.ExecuteACO(ref env);

                    ////Store variables in a txt
                    Console.WriteLine("Execution {0} of " + file_name, j);
                    StoreVariables_RW2(mesh_type, ref env, ref rw2);
                    StoreVariable_RW2_and_ACOv0(mesh_type, ref env, ref acov0, ref rw2);
                    env.InitPheromones(0);

                }

                Console.WriteLine("End Execution -> " + file_name);
                Console.WriteLine("---------------------------------------------");

            }



        }
    }
}
