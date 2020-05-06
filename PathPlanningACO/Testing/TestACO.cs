using PathPlanningACO.ACO;
using PathPlanningACO.EnvironmentProblem;
using PathPlanningACO.OtherMethods.Dijkstra;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace PathPlanningACO.Testing
{
    class TestACO
    {
        public static int num_test = 30;

        //--------------------------------------------------------------------
        public static void StoreTestingVariablesACOv0(string mesh_type, ref MeshEnvironment env, ref AntColonyOptimizationv0 aco)
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
            line += percentage_learning;

            string path = @"data_sets/results/acov0/";
            string variables_file = "variables_acov0" + "_" + mesh_type + "_" + env.size + "x" + env.size + ".txt";

            using (StreamWriter file =
             File.AppendText(path + variables_file))
            {
                file.WriteLine(line);
            }
        }

        //--------------------------------------------------------------------
        public static void TestACOv0(string mesh_type)
        {

            int[] sizes_mesh = new int[21] { 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50 };
            int[] final_positions = new int[21] { 99, 143, 195, 255, 323, 399, 483, 575, 675, 783, 899, 1023, 1155, 1295, 1443, 1599, 1763, 1935, 2115, 2303, 2499 };

            for (int i = 0; i < sizes_mesh.Length; i++)
            {
                string size = "_" + sizes_mesh[i] + "x" + sizes_mesh[i] + ".txt";
                string file_name = mesh_type + size;
                int j = 0;
                Console.WriteLine("Execution -> " + file_name);

                while (j < num_test)
                {
                    //Create the mesh environment
                    MeshEnvironment env = new MeshEnvironment(_start: 0, _final: final_positions[i], _file_name: file_name, sizes_mesh[i]);
                    env.InitEnviroment(type_mesh: mesh_type);
                    //-------------------------------------------------------------------

                    //Create and Execute ACOv0
                    AntColonyOptimizationv0 acov0 = new AntColonyOptimizationv0();
                    acov0.ExecuteACO(ref env);

                    //-------------------------------------------------------------------
                    //Store variables in a txt
                    Console.WriteLine("Execution {0}", j);
                    StoreTestingVariablesACOv0(mesh_type, ref env, ref acov0);
                    j++;
                    //}

                }
            }


        }
    }
}
