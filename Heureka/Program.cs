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

/*
 * NOTE TO SELF: USE HEURISTIC FOR CLAUSES
 */

namespace Heureka
{
    class Program
    {
        static void Main(string[] args)
        {
            //RouteFinding();
            Inference();
        }

        static void RouteFinding()
        {
            // Constructs a new graph from specified file
            var graph = Wrapper.MapFromFile("C:/AI/manhattan.txt");

            // Solves the Route Finding problem
            var solution = Solve(graph, "street_6", "avenue_7");

            // Print route to Console (optionally file)
            //var filepath = "";
            var filepath = "C:/Users/nickl/Desktop/route.txt";
            Wrapper.Print(solution, filepath);

            // If printing to file, open it, else leave Console open
            Wrapper.OpenFileIfExists(filepath);
        }

        static void Inference()
        {
            var engine = Wrapper.EngineFromFile("C:/AI/inference.txt");

            var clause = "a";

            if (engine.Validate(clause))
                Console.WriteLine("VALID");
            else
                Console.WriteLine("UNPROVEN");

            //// Print route to Console (optionally file)
            //var filepath = "";
            ////var filepath = "C:/Users/nickl/Desktop/route.txt";
            //Wrapper.Print(solution, filepath);

            //// If printing to file, open it, else leave Console open
            //Wrapper.OpenFileIfExists(filepath);
            Console.ReadKey();
        }

        static List<string> Solve(Graph graph, string edge1, string edge2)
        {
            // Where are we?
            var pathfinder = new AStar(graph, graph.GetNodeFromEdgeIdentifiers(edge1, edge2));

            // Where to?
            return Wrapper.GetRouteAsList(pathfinder, 0, 0);
        }

    }

    #region Wrapper
    static class Wrapper
    {
        public static Graph MapFromFile(string file)
        {
            var constructor = new MapReader(file);
            return constructor.getGraph();
        }

        public static InferenceEngine EngineFromFile(string file)
        {
            var engine = new InferenceEngine(file);
            return engine;
        }

        public static void Print(List<String> route, string filepath = null)
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
            var path = pathfinder.Find(x, y, null);
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

    class MapReader : GraphConstructor
    {
        public MapReader(string filepath) : base(filepath) { }
    }

    #endregion

    #region Inference
    class Clause : Node
    {
        public Clause(string id) : base(id : id)
        {
            properties.group = new List<List<Edge>>();
        }
    }

    class InferenceEngine : GraphConstructor
    {
        const int UNPROVEN_DISTANCE = int.MaxValue - 1;

        public InferenceEngine(string filepath) : base(null)
        {
            properties.emptyclause = new Clause(Guid.NewGuid().ToString());
            properties.emptyclause.properties.heuristic = 0;
            string[] lines = System.IO.File.ReadAllLines(filepath);
            foreach (var line in lines)
            {
                Console.WriteLine("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
                var keys = line.Split(' ');
                if (keys.Length == 1 && !String.IsNullOrEmpty(keys[0]))
                {
                    var clause = AddClause(keys[0]);
                    AddEdgeToEmpty(clause);
                }
                else if (keys.Length > 1)
                {
                    var target = AddClause(keys[0]);
                    var group = new List<Edge>();
                    for(int i = 2; i < keys.Length; i++)
                    {
                        if (keys[i] != keys[0])
                        {
                            var clause = AddClause(keys[i]);
                            var edge = new Edge(target, clause);
                            edge.properties.clause = true;
                            group.Add(edge);
                            graph.AddEdge(edge);
                        }
                    }
                    if (group.Count > 0)
                        target.properties.group.Add(group);
                }
                foreach (var n in graph.nodes)
                {
                    var proven = (AttemptProof((Clause)n)) ? ", proven" : "";
                    var str = n.properties.identifier.ToString() + " (d:" + n.properties.heuristic + "" + proven + ")  ";
                    if (n.properties.group.Count > 0)
                        foreach (var g in n.properties.group)
                        {
                            str = str + "[ ";
                            foreach (var p in g)
                                str = str + p.end.properties.identifier.ToString() + " ";
                            str = str + "]";
                        }
                    Console.WriteLine(str);
                    foreach (var e in n.GetEdges())
                    {
                        var prop = e.end.properties.identifier.ToString();
                        var id = (prop != properties.emptyclause.properties.identifier.ToString()) ? prop : "E";
                        Console.WriteLine("- " + id);
                    }
                    Console.WriteLine();
                }
            }
        }

        public bool Validate(string line)
        {
            var keys = line.Split(' ');
            if (keys.Length > 0)
            {
                var clause = GetClauseIfExists(keys[0]);
                if (clause != null)
                {
                    return Validate(clause);
                }
            }
            return false;
        }

        public bool Validate(Clause clause)
        {
            var pathfinder = new AStar(graph, clause);
            var path = pathfinder.Find(clause:properties.emptyclause);
            var result = (path != null);
            if (result)
                return (path.Count > 0);
            return result;
        }

        public Clause GetClauseIfExists(string id)
        {
            var index = graph.nodes.FindIndex(e => e.properties.identifier == id);
            return (index >= 0) ? (Clause)graph.nodes[index] : null;
        }

        public Clause AddClause(string id)
        {
            var clause = GetClauseIfExists(id);
            if (clause == null)
            {
                clause = new Clause(id);
                clause.properties.heuristic = UNPROVEN_DISTANCE; // fix this value
                graph.AddNode(clause);
            }
            UpdateHeuristics();
            AttemptProof(clause);
            return clause;
        }

        public void AddEdgeToEmpty(Clause clause)
        {
            var edge = GetEmptyEdgeIfExists(clause);
            if (edge == null)
            {
                edge = new Edge(clause, properties.emptyclause);
                edge.properties.clause = true;
                edge.start.properties.heuristic = 1;
                graph.AddEdge(edge);
                AttemptProof(clause);
            }
        }

        public Edge GetEmptyEdgeIfExists(Clause clause)
        {
            return clause.GetEdgeToNode(properties.emptyclause);
        }

        public void UpdateHeuristics()
        {
            foreach (var edge in graph.edges)
            {
                if (edge.end.properties.heuristic + 1 < edge.start.properties.heuristic)
                    edge.start.properties.heuristic = edge.end.properties.heuristic + 1;

                if (edge.end.properties.heuristic >= edge.start.properties.heuristic && edge.end.properties.heuristic != UNPROVEN_DISTANCE)
                    edge.start.properties.heuristic = edge.end.properties.heuristic + 1;
            }
        }

        public bool AttemptProof(Clause clause)
        {
            if (clause.GetEdges().Count == 0)
                return false;
            else if (!clause.HasProperty("proven"))
            {
                var valid = Validate(clause);
                if (valid)
                    clause.properties.proven = valid;
                return valid;
            }
            return true;
        }
    }

    #endregion

    #region Search Algorithms
    class AStar : Pathfinder
    {
        public AStar(Graph graph, Node node = null, int? x = null, int? y = null, string id = null) : base(graph, node, x, y, id) { }

        public override List<Edge> Find(int x = 0, int y = 0, Clause clause = null)
        {
            var inf = (clause != null);
            var goal = (inf) ? clause : SearchNode(x, y);
            Dictionary<string, Node> parent = new Dictionary<string, Node>();
            if (graph.isNullOrEmpty() || pos == null || goal == null)
                return null;
            var frontier = new List<Node>();
            var visited = new List<Node>();
            frontier.Add(pos);
            pos.properties.distance = 0;
            while (frontier.Count > 0)
            {
                var node = RemoveCheapestNode(frontier, goal);
                if (node.Equals(goal))
                {
                    if (inf)
                        return new List<Edge>() { graph.edges[0] }; // Return actual path instead of dummy
                    else
                        return RetrievePath(parent, goal);
                }
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

                    // Grouped Search Implemented Here

                    //if (node.HasProperty("group"))
                    //    foreach (var group in node.properties.group)
                    //    {
                    //        var proven = true;
                    //        foreach(var edge in group)
                    //        {
                    //            if (!visited.Contains(edge.end))
                    //            {
                    //                if (!edge.end.HasProperty("proven"))
                    //                {

                    //                }

                    //            }
                    //        }
                    //    }


                    foreach (var edge in node.GetEdges())
                        if (frontier.Contains(edge.end) && edge.end.HasProperty("distance"))
                        {
                            if (edge.Length() < edge.end.properties.distance)
                            {
                                parent[edge.end.ToString()] = node;
                                edge.end.properties.distance = edge.start.properties.distance + edge.Length();
                            }
                        }
                        else
                        {
                            frontier.Add(edge.end);
                            if (!parent.ContainsKey(edge.end.ToString()))
                                parent[edge.end.ToString()] = node;
                            if (edge.start.HasProperty("distance"))
                                edge.end.properties.distance = edge.start.properties.distance + edge.Length();
                            else
                                edge.end.properties.distance = edge.Length();
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
                var euclidean = (nodes[i].HasProperty("heuristic")) ? nodes[i].properties.heuristic : nodes[i].EuclideanDistance(target);
                var distance = nodes[i].properties.distance;
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

        public override List<Edge> Find(int x = 0, int y = 0, Clause clause = null)
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

        public Pathfinder(Graph graph, Node node = null, int? x = null, int? y = null, string id = null)
        {
            Init(graph);
            if (node != null)
                pos = node;
            else if (x.HasValue && y.HasValue)
                pos = SearchNode(x.Value, y.Value);
            else
                pos = SearchNode("id", id);
        }

        protected void Init(Graph graph)
        {
            this.graph = graph;
            properties.distance = new Dictionary<string, double>();
        }

        public Node SearchNode(string property, string str)
        {
            foreach (var node in graph.nodes)
                try
                {
                    var res = node.properties.GetType().GetProperty(property).GetValue(node.properties, null);
                    if (res.equals(str))
                        return node;
                }
                catch (Exception e) { }
            return null;
        }

        public Node SearchNode(int x, int y)
        {
            foreach (var node in graph.nodes)
                if (node.x == x && node.y == y)
                    return node;
            return null;
        }

        public abstract List<Edge> Find(int x, int y, Clause clause);

    }

    abstract class GraphConstructor
    {
        protected Graph graph = new Graph();
        protected dynamic properties = new ExpandoObject();

        public GraphConstructor(string filepath)
        {
            if (filepath == null)
                return;

            string[] lines = System.IO.File.ReadAllLines(filepath);
            foreach(var line in lines)
            {
                var keys = line.Split(' ');
                Node start = null, end = null;
                Edge edge = null;
                if (keys.Length == 5)
                {
                    start = new Node(Convert.ToDouble(keys[0]), Convert.ToDouble(keys[1]));
                    end = new Node(Convert.ToDouble(keys[3]), Convert.ToDouble(keys[4]));
                    edge = new Road(start, end, keys[2]);
                } else if (keys.Length == 3)
                {
                    start = new Node(id: keys[0]);
                    end = new Node(id: keys[2]);
                    edge = new Road(start, end, keys[1]);
                }
                if (start != null && end != null && edge != null)
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

        public Node(double? x = null, double? y = null, string id = null, Edge edge = null)
        {
            if (x.HasValue && y.HasValue)
            {
                this.x = x.Value;
                this.y = y.Value;
            }
            else if (!String.IsNullOrEmpty(id))
                properties.identifier = id;
            else
                properties.identifier = Guid.NewGuid();
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
            try
            {
                return (x == node.x && y == node.y && properties.identifier == node.properties.identifier);
            }
            catch (Exception e)
            {
                return false;
            }
        }

        public bool HasIdentifier()
        {
            var id = properties.GetType().GetProperty("identifier");
            return (properties.GetType().GetProperty("identifier") != null);
        }

        public bool HasProperty(string property)
        {
            return (properties.GetType().GetProperty(property) != null);
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
            return (!HasProperty("clause")) ? start.EuclideanDistance(end) : start.properties.distance + 1;
        }

        public bool HasProperty(string property)
        {
            return (properties.GetType().GetProperty(property) != null);
        }

        public override string ToString()
        {
            return properties.identifier;
        }
    }
    #endregion

}