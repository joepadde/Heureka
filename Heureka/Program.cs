using System;
using System.Collections.Generic;
using System.Dynamic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

/*
 * Heureka 2017
 * 02180 Introduction to Artificial Intelligence
 * Asbjorn Kruuse, Nicklas Hansen
 */

/*
 * SEARCH STRING FOR UNIMPLEMENTED CODE:
 * 'TO BE IMPLEMENTED'
 */

namespace Heureka
{
    class Program
    {
        static void Main(string[] args)
        {
            // Constructs a new graph from specified file
            var graph = Wrapper.GraphFromFile("C:/AI/manhattan.txt");

            // Solves the Road Finding Problem
            var route = RoadFinding(graph);

            // Solves the Inference Problem
            //var route = InferenceEngine(graph);

            // Print route to Console (optionally file)
            //var filepath = "";
            var filepath = "C:/Users/nickl/Desktop/route.txt";
            Wrapper.PrintRoute(route, filepath);

            // If printing to file, open it, else leave Console open
            Wrapper.OpenFileIfExists(filepath);
        }

        static List<string> RoadFinding(Graph graph)
        {
            // Where are we?
            var pathfinder = new AStar(graph, graph.GetNodeFromEdgeIdentifiers("street_6", "avenue_7"));

            // Where to?
            return Wrapper.GetRouteAsList(pathfinder, 0, 0);
        }

        static List<string> InferenceEngine(Graph graph)
        {
            // TO BE IMPLEMENTED
            return new List<string>();
        }

    }

    #region Wrapper
    static class Wrapper
    {
        public static Graph GraphFromFile(string file)
        {
            var constructor = new GraphConstructor(file);
            return constructor.getGraph();
        }

        public static void PrintRoute(List<String> route, string filepath = null)
        {
            if (route == null)
                return;
            foreach (var street in route)
                Console.WriteLine(street);
            if (!String.IsNullOrEmpty(filepath))
                try
                {
                    System.IO.File.WriteAllLines(filepath, route);
                }
                catch (Exception e)
                {
                    Console.WriteLine("\nERROR: Could not write to file");
                }
        }

        public static List<String> GetRouteAsList(Pathfinder pathfinder, int x, int y)
        {
            var route = new List<String>();
            var path = pathfinder.Find(x, y);
            if (path != null)
                foreach (var edge in path)
                    route.Add(edge.properties.identifier);
            else
                route.Add("Destination cannot be reached!");
            return route;
        }

        public static void OpenFileIfExists(string filepath)
        {
            if (!String.IsNullOrEmpty(filepath))
                System.Diagnostics.Process.Start(filepath);
            else
                Console.ReadKey();
        }
    }
    #endregion

    #region Route Finding
    class Road : Edge
    {
        public Road(Node start, Node end, string street) : base(start, end)
        {
            properties.identifier = street;
        }
    }
    #endregion

    #region Inference
    // TO BE IMPLEMENTED
    #endregion

    #region Search Algorithms
    class AStar : Pathfinder
    {

        public AStar(Graph graph, Node node) : base(graph, node) { }

        public override List<Edge> Find(int x, int y)
        {
            var goal = SearchNode(x, y);
            Dictionary<string, Node> parent = new Dictionary<string, Node>();
            if (graph.isNullOrEmpty() || pos == null || goal == null)
                return null;
            var frontier = new List<Node>();
            var visited = new List<Node>();
            frontier.Add(pos);
            properties.distance[pos.ToString()] = 0;
            while (frontier.Count > 0)
            {
                var node = RemoveCheapestNode(frontier, goal);
                if (node.Equals(goal))
                    return RetrievePath(parent, goal);
                if (node.GetEdges().Count == 0)
                    foreach (var n in graph.nodes)
                        if (n.Equals(node))
                        {
                            node = n;
                            break;
                        }
                if (!visited.Contains(node))
                {
                    visited.Add(node);
                    foreach (var edge in node.GetEdges())
                        if (frontier.Contains(edge.end) && properties.distance.ContainsKey(edge.end.ToString()))
                        {
                            if (edge.Length() < properties.distance[edge.end.ToString()])
                            {
                                parent[edge.end.ToString()] = node;
                                properties.distance[edge.end.ToString()] = properties.distance[edge.start.ToString()] + edge.Length();
                            }
                        }
                        else
                        {
                            frontier.Add(edge.end);
                            if (!parent.ContainsKey(edge.end.ToString()))
                                parent[edge.end.ToString()] = node;
                            properties.distance[edge.end.ToString()] = properties.distance[edge.start.ToString()] + edge.Length();
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
                try
                {
                    var p = parent[node.ToString()];
                    path.Add(p.GetEdgeToNode(node));
                    node = p;
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.StackTrace);
                    return new List<Edge>();
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
            for (int i = 0; i < nodes.Count; i++)
            {
                var euclidean = nodes[i].EuclideanDistance(target);
                var distance = properties.distance[nodes[i].ToString()];
                var total = euclidean + distance;
                if (total < cost || cost == -1)
                {
                    cost = total;
                    index = i;
                }
            }
            return index;
        }
    }

    class RBFS : Pathfinder
    {
        public RBFS(Graph graph, Node node) : base(graph, node) { }

        public override List<Edge> Find(int x, int y)
        {   
            // TO BE IMPLEMENTED
            throw new NotImplementedException();
        }
    }

    #endregion

    #region General Purpose
    abstract class Pathfinder
    {
        protected Graph graph;
        protected Node pos;
        protected dynamic properties = new ExpandoObject();

        public Pathfinder(Graph graph, Node node)
        {
            Init(graph);
            pos = node;
        }

        public Pathfinder(Graph graph, int x, int y)
        {
            Init(graph);
            pos = SearchNode(x, y);
        }

        protected void Init(Graph graph)
        {
            this.graph = graph;
            properties.distance = new Dictionary<string, double>();
        }

        public Node SearchNode(int x, int y)
        {
            foreach (var node in graph.nodes)
                if (node.x == x && node.y == y)
                    return node;
            return null;
        }

        public abstract List<Edge> Find(int x, int y);
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
                if (keys.Length == 5)
                {
                    var start = new Node(Convert.ToDouble(keys[0]), Convert.ToDouble(keys[1]));
                    var end = new Node(Convert.ToDouble(keys[3]), Convert.ToDouble(keys[4]));
                    var edge = new Road(start, end, keys[2]);

                    if (!line.Equals(lines[0]))
                    {
                        bool addStart = true, addEnd = true;
                        Node n = null;
                        foreach (var node in graph.nodes)
                        {
                            if (node.Equals(start))
                            {
                                addStart = false;
                                n = node;
                            }
                            if (node.Equals(end))
                                addEnd = false;
                        }
                        if (addStart)
                            graph.AddNode(start);
                        if (addEnd)
                            graph.AddNode(end);
                        if (n != null)
                            edge.start = n;
                        graph.AddEdge(edge);
                    }
                    else
                    {
                        graph.AddNode(start);
                        graph.AddNode(end);
                        graph.AddEdge(edge);
                    }
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

        public Node GetNodeFromEdgeIdentifiers(string one, string two)
        {
            foreach (var node in nodes)
            {
                int i = node.GetEdges().FindIndex(e => e.properties.identifier == one);
                int j = node.GetEdges().FindIndex(e => e.properties.identifier == two);
                if (i >= 0 && j >= 0)
                    return node;
            }

            return null;
        }
    }

    class Node
    {
        public double x, y;
        protected List<Edge> edges = new List<Edge>();
        public dynamic properties = new ExpandoObject();

        public Node(double x, double y, Edge edge = null)
        {
            this.x = x;
            this.y = y;
            if (edge != null)
                AddEdge(edge);
        }

        public void AddEdge(Edge edge)
        {
            edges.Add(edge);
        }

        public Edge GetEdgeToNode(Node node)
        {
            foreach(var edge in edges)
                if (edge.end.Equals(node))
                    return edge;
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
        public dynamic properties = new ExpandoObject();

        public Edge(Node start, Node end)
        {
            this.start = start;
            this.end = end;
            properties.identifier = Guid.NewGuid().ToString();
        }

        public bool Equals(Edge edge)
        {
            return (start.Equals(edge.start) && end.Equals(edge.end) && properties.identifier.Equals(edge.properties.identifier));
        }

        public double Length()
        {
            return start.EuclideanDistance(end);
        }

        public override string ToString()
        {
            return properties.identifier;
        }
    }
    #endregion

}