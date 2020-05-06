using System;
using System.Collections.Generic;
using System.Text;

namespace PathPlanningACO.EnvironmentProblem
{
    public class Edge : IEquatable<Edge>
    {
        public int id;
        public Node node1;
        public Node node2;
        public Double distance;
        public Double pheromone_amount;
        public Double pheromone_stuck;
        public bool visited;
        //-------------------------------------------------------------------

        public Edge(int _id, Node _node1, Node _node2)
        {
            id = _id;
            node1 = _node1;
            node2 = _node2;
            distance = 0;
            pheromone_amount = 0;
            pheromone_stuck = 0;
            visited = false;
        }

        //-------------------------------------------------------------------

        public Edge(Node _node1, Node _node2)
        {
            id = -1;
            node1 = _node1;
            node2 = _node2;
            distance = 0;
            pheromone_amount = 0;
            pheromone_stuck = 0;
            visited = false;
        }

        //-------------------------------------------------------------------

        public Edge(Node _node1, Node _node2, Double _pheromone_amount)
        {
            id = -1;
            node1 = _node1;
            node2 = _node2;
            distance = 0;
            pheromone_amount = _pheromone_amount;
            pheromone_stuck = 0;
            visited = false;
        }
        //-------------------------------------------------------------------

        public Edge(int _id, Node _node1, Node _node2, Double _distance)
        {
            id = _id;
            node1 = _node1;
            node2 = _node2;
            distance = _distance;
            pheromone_amount = 0;
            pheromone_stuck = 0;
            visited = false;
        }
        //-------------------------------------------------------------------

        public Edge(int _id, Node _node1, Node _node2, Double _distance, Double _pheromone_amount)
        {
            id = _id;
            node1 = _node1;
            node2 = _node2;
            distance = _distance;
            pheromone_amount = _pheromone_amount;
            pheromone_stuck = 0;
            visited = false;
        }
        //-------------------------------------------------------------------

        public override string ToString()
        {
            return "id: " + id + " | pheromone: " + pheromone_amount + " | pheromone stuck: " + pheromone_stuck;
        }
        //-------------------------------------------------------------------

        public bool Equals(Edge other)
        {
            bool result = (node1 == other.node1 & node2 == other.node2) | (node1 == other.node2 & node2 == other.node1);
            return result;
        }

        //-------------------------------------------------------------------

    }

}
