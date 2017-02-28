using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Heureka
{
    class Program
    {
        static void Main(string[] args)
        {
            var constructor = new GraphConstructor("C:/AI/copenhagen.txt");
            var graph = constructor.getGraph();
            Console.WriteLine("\n[Graph size: " + graph.nodes.Count + "]");
            Console.WriteLine("[Edge count: " + graph.edges.Count + "]");

            // Where Are We?
            var pathfinder = new Pathfinder(graph, 10, 70);

            // Where To?
            var path = pathfinder.Find(35, 120);

            if (path != null)
            {
                Console.WriteLine("\nFound path to destination!\n");
                foreach(var edge in path)
                {
                    Console.WriteLine(edge.ToString());
                }
            }
            Console.ReadKey();
        }
    }

    class Pathfinder
    {
        Graph graph;
        Node pos;
        Dictionary<string, double> distance = new Dictionary<string, double>();

        public Pathfinder(Graph graph, int x, int y)
        {
            this.graph = graph;
            pos = SearchNode(x, y);
        }

        public List<Edge> Find(int x, int y)
        {
            var goal = SearchNode(x, y);
            Dictionary<string, Node> parent = new Dictionary<string, Node>();

            if (graph.isNullOrEmpty() || pos == null || goal == null)
                return null;
            Console.WriteLine("Initiating search...");

            var frontier = new List<Node>();
            var visited = new List<Node>();
            frontier.Add(pos);
            distance.Add(pos.ToString(), 0);
            while(frontier.Count > 0)
            {
                var node = RemoveCheapestNode(frontier, goal);
                if (node.Equals(goal))
                    return RetrievePath(parent, goal);
                Console.WriteLine("\nStanding at " + node.ToString());
                if (!visited.Contains(node))
                {
                    visited.Add(node);
                    Console.WriteLine("Current node has " + node.GetEdges().Count + " edges");
                    foreach(var edge in node.GetEdges())
                    {
                        // TODO A* cases
                        //if (frontier.Contains(edge.end))
                        //{

                        //}
                        Console.WriteLine("Adding node " + edge.end.x + "," + edge.end.y + " to OPEN");
                        frontier.Add(edge.end);
                        try
                        {
                            parent.Add(edge.end.ToString(), node);
                            distance.Add(edge.end.ToString(), edge.Length());
                        } catch (Exception e)
                        {
                            Console.WriteLine(e.StackTrace);
                        }
                    }
                }
            }
            return null;
        }

        public List<Edge> RetrievePath(Dictionary<string, Node> parent, Node goal)
        {
            var path = new List<Edge>();
            var node = goal;
            while (node != pos)
            {
                try
                {
                    var p = parent[node.ToString()];
                    path.Add(p.GetEdgeToNode(node));
                    node = p;
                } catch (Exception e)
                {
                    Console.WriteLine(e.StackTrace);
                    return new List<Edge>();
                }
            }
            path.Reverse();
            return path;
        }

        public Node RemoveCheapestNode(List<Node> nodes, Node target)
        {
            var index = IndexOfCheapestNode(nodes, target);
            var node = nodes[index];
            nodes.RemoveAt(index);
            return node;
        }

        public int IndexOfCheapestNode(List<Node> nodes, Node target)
        {
            double cost = -1;
            int index = 0;
            for(int i = 0; i < nodes.Count; i++)
            {
                var euclidean = nodes[i].EuclideanDistance(target);
                var distance = this.distance[nodes[i].ToString()];
                if (euclidean + distance < cost || cost == -1)
                {
                    cost = euclidean;
                    index = i;
                }
            }
            return index;
        }

        public Node SearchNode(int x, int y)
        {
            foreach (var node in graph.nodes)
            {
                if (node.x == x && node.y == y)
                {
                    return node;
                }
            }
            return null;
        }
    }


    class GraphConstructor
    {
        protected Graph graph = new Graph();

        public GraphConstructor(string filepath)
        {
            string[] lines = System.IO.File.ReadAllLines(filepath);
            foreach(var line in lines)
            {
                var keys = line.Split(' ');
                var start = new Node(Convert.ToDouble(keys[0]), Convert.ToDouble(keys[1]));
                var end = new Node(Convert.ToDouble(keys[3]), Convert.ToDouble(keys[4]));
                var edge = new Edge(start, end, keys[2]);
                start.AddEdge(edge);

                if (!line.Equals(lines[0]))
                {
                    bool addStart = true, addEnd = true;
                    foreach (var node in graph.nodes)
                    {
                        if (node.Equals(start))
                        {
                            addStart = false;
                            node.AddEdge(edge);
                        }
                        if (node.Equals(end))
                            addEnd = false;
                    }
                    if (addStart)
                        graph.AddNode(start);
                    if (addEnd)
                        graph.AddNode(end);
                    graph.AddEdge(edge);
                } else
                {
                    graph.AddNode(start);
                    graph.AddNode(end);
                    graph.AddEdge(edge);
                }
            }
        }

        public Graph getGraph()
        {
            return graph;
        }
    }

    class Graph
    {
        public List<Node> nodes = new List<Node>();
        public List<Edge> edges = new List<Edge>();

        public Graph() { }

        public void AddNode(Node node)
        {
            nodes.Add(node);
        }

        public void AddEdge(Edge edge)
        {
            edges.Add(edge);
            edge.start.AddEdge(edge);
        }

        public bool isNullOrEmpty()
        {
            if (this == null)
                return true;

            return (nodes.Count == 0 || edges.Count == 0);
        }
    }

    class Node
    {
        public double x, y;
        protected List<Edge> edges = new List<Edge>();

        public Node(double x, double y, Edge edge = null)
        {
            this.x = x;
            this.y = y;
            if (edge != null)
            {
                AddEdge(edge);
            }
        }

        public void AddEdge(Edge edge)
        {
            edges.Add(edge);
        }

        public Edge GetEdgeToNode(Node node)
        {
            foreach(var edge in edges)
            {
                if (edge.end.Equals(node))
                    return edge;
            }
            return null;
        }

        public List<Edge> GetEdges()
        {
            return edges;
        }      

        public double EuclideanDistance(Node target)
        {
            return Math.Sqrt(Math.Pow(x - target.x, 2) + Math.Pow(y - target.y, 2));
        }

        public bool Equals(Node node)
        {
            return (x == node.x && y == node.y);
        }

        public override string ToString()
        {
            return "[" + x + "," + y + "]";
        }
    }

    class Edge
    {
        public Node start;
        public Node end;
        public string street;

        public Edge(Node start, Node end, string street)
        {
            this.start = start;
            this.end = end;
            this.street = street;
        }

        public bool Equals(Edge edge)
        {
            return (start.Equals(edge.start) && end.Equals(edge.end) && street.Equals(edge.street));
        }

        public override string ToString()
        {
            //return start.ToString() + " " + street + " " + end.ToString();
            return street;
        }

        public double Length()
        {
            return start.EuclideanDistance(end);
        }
    }
}