using PathPlanningACO.Testing;
using System;

namespace PathPlanningACO
{
    class Program
    {
        static void Main(string[] args)
        {
            string[] mesh_types = new string[4] { "mountain", "valley", "doble_valley", "perlin" };

            //-------------Testing ACOv0 alone--------------------------

            //for (int i = 0; i < mesh_types.Length; i++)
            //{
            //    TestACO.TestACOv0(mesh_types[i]);
            //}

            //-------------Testing ACOv0 + Random Walk 1-----------------
            //for (int i = 0; i < mesh_types.Length; i++)
            //{
            //    TestRandomWalksACO.TestRandomWalkv1(mesh_types[i]);
            //}

            //-------------Testing ACOv0 + Random Walk 2-----------------
            //for (int i = 0; i < mesh_types.Length; i++)
            //{
            //    TestRandomWalksACO.TestRandomWalkv2(mesh_types[i]);
            //}

            //-----------------------Several Methods -------------------
            //---------------------Dijkstra Algorrithm------------------
            //for (int i = 0; i < mesh_types.Length; i++)
            //{
            //    TestSeveralMethods.TestDijkstra(mesh_types[i]);
            //}

            //-----------------------A* Algorrithm------------------
            //for (int i = 0; i < mesh_types.Length; i++)
            //{
            //    TestSeveralMethods.TestAstar(mesh_types[i]);
            //}

            //-----------------------Genetic Algorrithm------------------
            //for (int i = 0; i < mesh_types.Length; i++)
            //{
            //    TestSeveralMethods.Test_Genetic(mesh_types[i]);
            //}
            //-----------------------ACOv0 + RW 2  (Route 1 and 2)------------------
            for (int i = 0; i < mesh_types.Length; i++)
            {
                TestSeveralMethods.Test_RWv2_ACOv0(mesh_types[i]);
            }

        }
    }
}
