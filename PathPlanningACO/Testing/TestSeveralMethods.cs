using PathPlanningACO.ACO;
using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.OtherMethods.A_star;
using PathPlanningACO.OtherMethods.Dijkstra;
using PathPlanningACO.OtherMethods.Genetic;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace PathPlanningACO.Testing
{
    class TestSeveralMethods
    {
        static int num_test = 30;

        public static void InsertSeveralObstacle(ref MeshEnvironment env, int num_obstacle, int num_routes)
        {
            for (int i = 0; i < num_routes; i++)
            {
                env.FillDistanceMatrix();
                DijkstraAlgorithm dk = new DijkstraAlgorithm();
                dk.Execute(ref env);

                //Ver bien como poner esto
                env.InsertObstacle(ref dk.final_best_path, num_obstacle); 
            }

        }
        //-------------------------------------------------------------------------------
        public static void StoreVariable_RWv2_ACOv0(string mesh_type, ref MeshEnvironment env, ref DijkstraAlgorithm dk1, ref AntColonyOptimizationv0 aco1, ref DijkstraAlgorithm dk2, ref AntColonyOptimizationv0 aco2)
        {
            string separator = " ";
            string method = "rw2_acov0";

            Double d2 = aco2.best_cost < dk2.final_best_cost ? dk2.final_best_cost : aco2.best_cost;
            Double ac1 = (dk1.final_best_cost / aco1.best_cost) * 100;
            Double ac2 = (dk2.final_best_cost / d2) * 100;

            string line = mesh_type + separator;
            line += env.size + separator;
            line += dk1.final_best_cost + separator;
            line += aco1.best_cost + separator;
            line += ac1 + separator;
            line += aco1.execution_time + separator;
            line += dk2.final_best_cost + separator;
            line += d2 + separator;
            line += ac2 + separator;
            line += aco2.execution_time;


            string path2 = @"data_sets/results/several_methods/";
            string variables_file2 = method + "_" + mesh_type + ".txt";

            using (StreamWriter file =
                File.AppendText(path2 + variables_file2))
            {
                file.WriteLine(line);
            }

            string path = @"data_sets/results/several_methods/" + mesh_type + "/";
            string variables_file = method + "_" + mesh_type + "_" + env.size + "x" + env.size + ".txt";

            using (StreamWriter file =
             File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }
        }
        public static void Test_RWv2_ACOv0(string mesh_type)
        {

            int[] sizes_mesh = new int[21] { 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50 };
            int[] final_positions = new int[21] { 99, 143, 195, 255, 323, 399, 483, 575, 675, 783, 899, 1023, 1155, 1295, 1443, 1599, 1763, 1935, 2115, 2303, 2499 };

            for (int i = 0; i < sizes_mesh.Length; i++)
            {
                string size = "_" + sizes_mesh[i] + "x" + sizes_mesh[i] + ".txt";
                string file_name = mesh_type + size;

                Console.WriteLine("Init Execution -> " + file_name);

                //-------------------------------------------------------------------
                //Create the mesh environment
                MeshEnvironment env = new MeshEnvironment(_start: 0, _final: final_positions[i], _file_name: file_name, sizes_mesh[i]);
                env.InitEnviroment(type_mesh: mesh_type);

                //-------------------------------------------------------------------
                for (int j = 0; j < num_test; j++)
                {
                    //............................................................................
                    // ROUTE 1
                    env.FillDistanceMatrix();
                    DijkstraAlgorithm dk1 = new DijkstraAlgorithm();
                    dk1.Execute(ref env);

                    RandomWalkv2 rw1 = new RandomWalkv2();
                    rw1.ExecuteRandomWalk(ref env);

                    AntColonyOptimizationv0 aco1 = new AntColonyOptimizationv0(_random_walk: true);
                    aco1.ExecuteACO(ref env);

                    //............................................................................
                    //Insert the obstacle in a middle zone of the current optimal solution
                    InsertSeveralObstacle(ref env, num_obstacle: 2, num_routes: 2);
                    //............................................................................
                    // ROUTE 2 - REROUTING

                    env.FillDistanceMatrix();
                    DijkstraAlgorithm dk2 = new DijkstraAlgorithm();
                    dk2.Execute(ref env);

                    RandomWalkv2 rw2 = new RandomWalkv2();
                    rw2.ExecuteRandomWalk(ref env);

                    AntColonyOptimizationv0 aco2 = new AntColonyOptimizationv0(_random_walk: true);
                    aco2.ExecuteACO(ref env);

                    //............................................................................
                    ////Store variables in a txt
                    Console.WriteLine("Execution {0} of " + file_name, j);
                    StoreVariable_RWv2_ACOv0(mesh_type, ref env, ref dk1, ref aco1, ref dk2, ref aco2);

                    //............................................................................
                    //Reset environment
                    env.InitPheromones(0);
                    env.ClearObstacles();

                }

                Console.WriteLine("End Execution -> " + file_name);
                Console.WriteLine("---------------------------------------------");

            }
        }

        //-------------------------------------------------------------------------------
        public static void StoreVariable_Genetic(string mesh_type, ref MeshEnvironment env, ref DijkstraAlgorithm dk1, ref GeneticAlgorithm gn1, ref DijkstraAlgorithm dk2, ref GeneticAlgorithm gn2)
        {

            string separator = " ";
            string method = "genetic";
            Double d2 = gn2.final_best_cost < dk2.final_best_cost ? dk2.final_best_cost : gn2.final_best_cost;
            Double ac1 = (dk1.final_best_cost / gn1.final_best_cost) * 100;
            Double ac2 = (dk2.final_best_cost / d2) * 100;

            string line = mesh_type + separator;
            line += env.size + separator;
            line += dk1.final_best_cost + separator;
            line += gn1.final_best_cost + separator;
            line += ac1 + separator;
            line += gn1.execution_time + separator;
            line += dk2.final_best_cost + separator;
            line += d2 + separator;
            line += ac2 + separator;
            line += gn2.execution_time;

            string path = @"data_sets/results/several_methods/" + mesh_type + "/";
            string variables_file = method + "_" + mesh_type + "_" + env.size + "x" + env.size + ".txt";

            using (StreamWriter file =
             File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }

            string path2 = @"data_sets/results/several_methods/";
            string variables_file2 = method + "_" + mesh_type + ".txt";

            using (StreamWriter file =
                File.AppendText(path2 + variables_file2))
            {
                file.WriteLine(line);
            }
        }
        public static void Test_Genetic(string mesh_type)
        {
            int[] sizes_mesh = new int[21] { 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50 };
            int[] final_positions = new int[21] { 99, 143, 195, 255, 323, 399, 483, 575, 675, 783, 899, 1023, 1155, 1295, 1443, 1599, 1763, 1935, 2115, 2303, 2499 };

            for (int i = 0; i < sizes_mesh.Length; i++)
            {
                string size = "_" + sizes_mesh[i] + "x" + sizes_mesh[i] + ".txt";
                string file_name = mesh_type + size;

                Console.WriteLine("Init Execution -> " + file_name);

                //-------------------------------------------------------------------
                //Create the mesh environment
                MeshEnvironment env = new MeshEnvironment(_start: 0, _final: final_positions[i], _file_name: file_name, sizes_mesh[i]);
                env.InitEnviroment(type_mesh: mesh_type);

                //-------------------------------------------------------------------
                for (int j = 0; j < num_test; j++)
                {
                    env.FillDistanceMatrix();
                    DijkstraAlgorithm dk1 = new DijkstraAlgorithm();
                    dk1.Execute(ref env);

                    GeneticAlgorithm gn1 = new GeneticAlgorithm();
                    gn1.Execute(ref env);


                    //Insert the obstacle in a middle zone of the current optimal solution
                    InsertSeveralObstacle(ref env, num_obstacle: 2, num_routes: 2);

                    env.FillDistanceMatrix();
                    DijkstraAlgorithm dk2 = new DijkstraAlgorithm();
                    dk2.Execute(ref env);

                    GeneticAlgorithm gn2 = new GeneticAlgorithm();
                    gn2.Execute(ref env);

                    ////Store variables in a txt
                    Console.WriteLine("Execution {0} of " + file_name, j);

                    StoreVariable_Genetic(mesh_type, ref env, ref dk1, ref gn1, ref dk2, ref gn2);

                    env.ClearObstacles();

                }

                Console.WriteLine("End Execution -> " + file_name);
                Console.WriteLine("---------------------------------------------");

            }
        }
        //-------------------------------------------------------------------------------
        public static void StoreVariable_Dijkstra(string mesh_type, ref MeshEnvironment env, ref DijkstraAlgorithm dk1, ref DijkstraAlgorithm dk2)
        {
            string separator = " ";
            string size = "" + env.size;
            string method = "dijkstra";

            string line = mesh_type + separator;
            line += env.size + separator;
            line += dk1.final_best_cost + separator;
            line += dk1.execution_time + separator;
            line += dk2.final_best_cost + separator;
            line += dk2.execution_time;

            string path = @"data_sets/results/several_methods/" + mesh_type + "/";
            string variables_file = method + "_" + mesh_type + "_" + size + "x" + size + ".txt";

            using (StreamWriter file =
                File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }

            string path2 = @"data_sets/results/several_methods/";
            string variables_file2 = method + "_" + mesh_type + ".txt";

            using (StreamWriter file =
                File.AppendText(path2 + variables_file2))
            {
                file.WriteLine(line);
            }


        }
        public static void TestDijkstra(string mesh_type)
        {
            int[] sizes_mesh = new int[21] { 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50 };
            int[] final_positions = new int[21] { 99, 143, 195, 255, 323, 399, 483, 575, 675, 783, 899, 1023, 1155, 1295, 1443, 1599, 1763, 1935, 2115, 2303, 2499 };

            for (int i = 0; i < sizes_mesh.Length; i++)
            {
                string size = "_" + sizes_mesh[i] + "x" + sizes_mesh[i] + ".txt";
                string file_name = mesh_type + size;

                Console.WriteLine("Init Execution -> " + file_name);

                //-------------------------------------------------------------------
                //Create the mesh environment
                MeshEnvironment env = new MeshEnvironment(_start: 0, _final: final_positions[i], _file_name: file_name, sizes_mesh[i]);
                env.InitEnviroment(type_mesh: mesh_type);



                //-------------------------------------------------------------------
                for (int j = 0; j < num_test; j++)
                {
                    DijkstraAlgorithm dk1 = new DijkstraAlgorithm();
                    dk1.Execute(ref env);

                    //Insert the obstacle in a middle zone of the current optimal solution
                    env.FillDistanceMatrix();
                    InsertSeveralObstacle(ref env, num_obstacle: 2, num_routes: 2);
                    env.FillDistanceMatrix();

                    DijkstraAlgorithm dk2 = new DijkstraAlgorithm();
                    dk2.Execute(ref env);

                    ////Store variables in a txt
                    Console.WriteLine("Execution {0} of " + file_name, j);
                    StoreVariable_Dijkstra(mesh_type, ref env, ref dk1, ref dk2);

                    env.ClearObstacles();

                }

                Console.WriteLine("End Execution -> " + file_name);
                Console.WriteLine("---------------------------------------------");

            }
        }
        //-------------------------------------------------------------------------------
        public static void StoreVariable_AStar(string mesh_type, ref MeshEnvironment env, ref AStarAlgorithm ast1, ref AStarAlgorithm ast2)
        {
            string separator = " ";
            string size = "" + env.size;
            string method = "astar";
            string line = mesh_type + separator;
            line += env.size + separator;
            line += ast1.final_best_cost + separator;
            line += ast1.execution_time + separator;
            line += ast2.final_best_cost + separator;
            line += ast2.execution_time;

            string path = @"data_sets/results/several_methods/" + mesh_type + "/";
            string variables_file = method + "_" + mesh_type + "_" + env.size + "x" + env.size + ".txt";

            using (StreamWriter file =
                File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }

            string path2 = @"data_sets/results/several_methods/";
            string variables_file2 = method + "_" + mesh_type + ".txt";

            using (StreamWriter file =
                File.AppendText(path2 + variables_file2))
            {
                file.WriteLine(line);
            }

        }
        public static void TestAstar(string mesh_type)
        {
            int[] sizes_mesh = new int[21] { 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50 };
            int[] final_positions = new int[21] { 99, 143, 195, 255, 323, 399, 483, 575, 675, 783, 899, 1023, 1155, 1295, 1443, 1599, 1763, 1935, 2115, 2303, 2499 };

            for (int i = 0; i < sizes_mesh.Length; i++)
            {
                string size = "_" + sizes_mesh[i] + "x" + sizes_mesh[i] + ".txt";
                string file_name = mesh_type + size;

                Console.WriteLine("Init Execution -> " + file_name);

                //-------------------------------------------------------------------
                //Create the mesh environment
                MeshEnvironment env = new MeshEnvironment(_start: 0, _final: final_positions[i], _file_name: file_name, sizes_mesh[i]);
                env.InitEnviroment(type_mesh: mesh_type);

                //-------------------------------------------------------------------
                for (int j = 0; j < num_test; j++)
                {
                    AStarAlgorithm as1 = new AStarAlgorithm();
                    as1.Execute(ref env);

                    //Insert the obstacle in a middle zone of the current optimal solution
                    env.FillDistanceMatrix();
                    InsertSeveralObstacle(ref env, num_obstacle: 2, num_routes: 2);
                    env.FillDistanceMatrix();

                    AStarAlgorithm as2 = new AStarAlgorithm();
                    as2.Execute(ref env);

                    ////Store variables in a txt
                    Console.WriteLine("Execution {0} of " + file_name, j);
                    StoreVariable_AStar(mesh_type, ref env, ref as1, ref as2);

                    env.ClearObstacles();

                }


                Console.WriteLine("End Execution -> " + file_name);
                Console.WriteLine("---------------------------------------------");

            }
        }
    }
}
