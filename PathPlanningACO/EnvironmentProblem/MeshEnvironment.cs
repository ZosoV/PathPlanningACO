using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Threading;

namespace PathPlanningACO.EnvironmentProblem
{
    class MeshEnvironment
    {
        //Attributes
        public string file_name;                        //Name of the file where the nodes are stored
        public List<Node> world = new List<Node>();     //Store all the nodes
        public List<Edge> edges = new List<Edge>();     //Sore all the edges
        public int size;                                //Size of the mesh
        public int start_node;      
        public int final_node;

        public List<int> obstacles = new List<int>();   //List that stores the ids of obstacles nodes
        public Double[,] distances;

        //-------------------------------------------------------------------

        //Initialize enviroment with a start and final node
        public MeshEnvironment(int _start, int _final, string _file_name, int _size)
        {
            start_node = _start;
            final_node = _final;
            file_name = _file_name;
            size = _size;
        }

        //-------------------------------------------------------------------

        //Init all the other attributes of the Enviroment
        public void InitEnviroment(string type_mesh)
        {
            world = new List<Node>(); //Store all the nodes
            edges = new List<Edge>(); //Sore all the edges

            string path = @"data_sets/" + type_mesh + "/";

            //Load data
            var file = File.ReadAllLines(path + file_name);


            //Init nodes, conections, and obstacles
            int count = 0;
            int count_edge = 0;
            for (int i = 2; i < file.Length; i++)
            {
                var fields = file[i].Split(' ');
                string identifier = fields[0];

                switch (identifier)
                {
                    //Init nodes
                    case "v":
                        Double x_coord = Convert.ToDouble(fields[1]);
                        Double y_coord = Convert.ToDouble(fields[2]);
                        Double z_coord = Convert.ToDouble(fields[3]);
                        world.Add(new Node(count, x_coord, y_coord, z_coord, 'N'));
                        count++;
                        break;

                    //Init connections in the heuristic_criteria matrix
                    case "f":

                        int node1, node2;
                        Edge new_edge;
                        Double distance;
                        for (int j = 1; j < fields.Length; j++)
                        {
                            node1 = Int32.Parse(fields[j]) - 1;
                            node2 = ((j == fields.Length - 1) ? Int32.Parse(fields[1]) : Int32.Parse(fields[j + 1])) - 1;
                            distance = ExtraTools.EucDistance(world[node1], world[node2]);

                            new_edge = new Edge(count_edge, world[node1], world[node2], distance);

                            if (!edges.Contains(new_edge))
                            {
                                edges.Add(new_edge);
                                world[node1].neighboors.Add(node2);
                                world[node1].edges.Add(edges.Count - 1);

                                world[node2].neighboors.Add(node1);
                                world[node2].edges.Add(edges.Count - 1);

                                count_edge++;
                            }
                        }

                        break;
                }


            }


            SetProximities();
        }
        //----------------------------------------------------------------
        public void InitPheromones(Double tau_0)
        {
            foreach (var edge in edges)
            {
                edge.pheromone_amount = tau_0;
            }

        }

        //-------------------------------------------------------------------
        //Inverse euclidean distance from the current node the target node
        public Double CalculateProximity(int current_node)
        {
            Double di_t = ExtraTools.EucDistance(world[current_node], world[final_node]);

            return di_t;
        }

        //-------------------------------------------------------------------
        //Proximity value selecting the target node
        public Double CalculateProximity(int current_node, int next_node, int target_node)
        {
            Double di_t = ExtraTools.EucDistance(world[current_node], world[target_node]);
            Double dj_t = ExtraTools.EucDistance(world[next_node], world[target_node]);

            return di_t / dj_t;
        }

        //-------------------------------------------------------------------
        //Proximity value setting the target node of the environment
        public Double CalculateProximity(int current_node, int next_node)
        {
            Double di_t = ExtraTools.EucDistance(world[current_node], world[final_node]);
            Double dj_t = ExtraTools.EucDistance(world[next_node], world[final_node]);

            return di_t / dj_t;
        }
        //-------------------------------------------------------------------
        public void GetMaxMinProximities(ref Double max_proximity, ref Double min_proximity)
        {
            max_proximity = Double.MinValue;
            min_proximity = Double.MaxValue;

            foreach (var node in world)
            {
                List<Double> proximities = new List<Double>();

                if (node.id != final_node)
                {
                    List<int> neighboors = node.neighboors;

                    foreach (var next_node in neighboors)
                    {
                        Double proximity = CalculateProximity(node.id, next_node);

                        if (next_node != final_node)
                        {
                            max_proximity = (proximity > max_proximity) ? proximity : max_proximity;
                            min_proximity = (proximity < min_proximity) ? proximity : min_proximity;
                        }

                    }
                }

            }
        }

        //-------------------------------------------------------------------
        public void SetProximities()
        {
            Double max_proximity = Double.MinValue;
            Double min_proximity = Double.MaxValue;

            GetMaxMinProximities(ref max_proximity, ref min_proximity);

            foreach (var node in world)
            {
                List<Double> proximities = new List<Double>();

                if (node.id != final_node)
                {
                    List<int> neighboors = node.neighboors;

                    foreach (var next_node in neighboors)
                    {
                        Double proximity = CalculateProximity(node.id, next_node);
                        proximity = ExtraTools.NormalizeWithInterval(proximity, min_proximity, max_proximity);

                        proximities.Add(proximity);


                    }
                }

                node.proximities = proximities;

            }


        }

        //-------------------------------------------------------------------
        public void ResetNodes()
        {
            for (int i = 0; i < world.Count; i++)
            {
                world[i].visited_by = -1;
            }
        }


        //-------------------------------------------------------------------
        public void FillDistanceMatrix()
        {
            distances = new double[world.Count, world.Count];

            for (int i = 0; i < edges.Count; i++)
            {
                Edge edge = edges[i];
                int node1_idx = edge.node1.id;
                int node2_idx = edge.node2.id;

                if (!obstacles.Contains(node1_idx) && !obstacles.Contains(node2_idx))
                {
                    distances[node1_idx, node2_idx] = edge.distance;
                    distances[node2_idx, node1_idx] = edge.distance;
                }

            }

        }


        //-------------------------------------------------------------------
        public void InsertObstacle(ref List<int> best_route, int num_obstacle)
        {

            int piece = best_route.Count / (num_obstacle + 1);

            for (int i = 0; i < num_obstacle; i++)
            {
                obstacles.Add(best_route[piece * (i + 1)]);

                foreach (var node_idx in world[best_route[piece * (i + 1)]].neighboors)
                {
                    obstacles.Add(node_idx);
                }
            }
        }
        //-------------------------------------------------------------------

        public void ClearObstacles()
        {
            obstacles = new List<int>();
        }

        //-------------------------------------------------------------------

        public void FillDistancesToFinal()
        {
            for (int i = 0; i < world.Count; i++)
            {
                world[i].distance_to_final = CalculateProximity(i);
                //world[i].distance_to_final = ExtraTools.ManhattanDistance(world[i], world[final_node]);
            }
        }

        //-------------------------------------------------------------------


    }

}
