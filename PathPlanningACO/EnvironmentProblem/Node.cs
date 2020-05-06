using PathPlanningACO.ExtraFunctions;
using System;
using System.Collections.Generic;
using System.Text;

namespace PathPlanningACO.EnvironmentProblem
{
    public class Node : IEquatable<Node>
    {
        //-------------------------------------------------------------------
        //---Attributes
        public int id;
        public Double X;
        public Double Y;
        public Double Z;


        public char mode; //O: obstacle, N: normal node, K: killer node
        public bool visited;

        public int visited_by; //By what ant?

        //List to store the neighboors node, and the respective edge conecctions
        public List<int> neighboors;
        public List<int> edges;

        //List to store the proximities ans slopes with the respective neighboors
        public List<Double> proximities;
        public List<Double> slopes;

        //List distance to final node
        public Double distance_to_final;

        //-------------------------------------------------------------------
        public Node(int _id, Double _x, Double _y, Double _z, char _mode)
        {
            id = _id;
            X = _x;
            Y = _y;
            Z = _z;
            mode = _mode;
            neighboors = new List<int>();
            edges = new List<int>();
            visited_by = -1;
            distance_to_final = 0;
            visited = false;
        }
        //-------------------------------------------------------------------
        public Node(Double _x, Double _y, Double _z)
        {
            id = -1;
            X = _x;
            Y = _y;
            Z = _z;
            mode = 'N';
            neighboors = new List<int>();
            edges = new List<int>();
            visited_by = -1;
            distance_to_final = 0;
            visited = false;
        }


        //-------------------------------------------------------------------
        public Node(int _id)
        {
            id = _id;
        }

        //-------------------------------------------------------------------

        public override string ToString()
        {
            if (mode == 'N')
            {
                string str_neigh = ExtraTools.PrintList(ref neighboors);
                string str_edges = ExtraTools.PrintList(ref edges);

                return "id: " + id + " neighboors: " + str_neigh + " edges: " + str_edges + " mode: " + mode;
            }
            else
            {
                return "obstacle";
            }

        }

        //-------------------------------------------------------------------

        public bool GetVisited(int current_ant)
        {
            return current_ant == visited_by;
        }

        //-------------------------------------------------------------------

        public bool Equals(Node other)
        {
            //bool result = (this.X == other.X & this.Y == other.Y & this.Z == other.Z);
            bool result = this.id == other.id;
            return result;
        }

        //-------------------------------------------------------------------

        public static bool operator ==(Node node1, Node node2)
        {
            return node1.Equals(node2);
        }

        //-------------------------------------------------------------------

        public static bool operator !=(Node node1, Node node2)
        {
            return !node1.Equals(node2);
        }
    }

}
