using System;
using System.Collections.Generic;
using System.Text;

namespace PathPlanningACO.OtherMethods.A_star
{
    //Estructura extra que pertenece a cada nodo
    //Esta estructura sirve para correr algoritmo A_star
    //o el Dijkstra

    public struct Tag : IEquatable<Tag>
    {
        public int node_id;
        public bool visited;        //flag para marcar si el nodo a sido visitado
        public Double global_goal;  //valor del objetivo global 
        public Double local_goal;   //valor del objetivo local
        public Nullable<int> parent;          //valor del nodo padre actual

        public Tag(int _node_id, bool _visited, Double _global_goal, Double _local_goal, Nullable<int> _parent)
        {
            node_id = _node_id;
            visited = _visited;
            global_goal = _global_goal;
            local_goal = _local_goal;
            parent = _parent;
        }

        public bool Equals(Tag other)
        {
            return node_id == other.node_id;
        }

        public override string ToString()
        {
            return "id: " + node_id + " | " + visited + " | (G: " + global_goal + " ,L: " + local_goal + " ,P: " + parent + ")";
        }
    }
}
