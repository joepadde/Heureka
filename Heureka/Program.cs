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

            Console.WriteLine("Graph size: " + graph.nodes.Count);

            foreach(var i in graph.nodes)
            {
                Console.WriteLine("Node: " + i.x + ", " + i.y);
            }

            Console.ReadKey();

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
                Console.WriteLine(line);
                var keys = line.Split(' ');

                var start = new Node(Convert.ToDouble(keys[0]), Convert.ToDouble(keys[1]));
                var end = new Node(Convert.ToDouble(keys[3]), Convert.ToDouble(keys[4]));
                var connection = new Edge(start, end, keys[2]);

                if (!line.Equals(lines[0]))
                {
                    foreach (var node in graph.nodes.AsEnumerable<Node>())
                    {
                        if (!node.Equals(start))
                            graph.AddNode(start);

                        if (!node.Equals(end))
                            graph.AddNode(end);
                    }

                    foreach (var edge in graph.edges)
                    {
                        if (!edge.Equals(connection))
                            graph.AddEdge(connection);

                    }
                } else
                {
                    graph.AddNode(start);
                    graph.AddNode(end);
                    graph.AddEdge(connection);
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
        }

    }

    class Node
    {
        public double x, y;
        protected List<Edge> edges;

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

        public bool Equals(Node node)
        {
            return (x == node.x && y == node.y);
        }

    }

    class Edge
    {
        public Node start;
        public Node end;
        public string street;

        public Edge(Node start, Node end, string street)
        {
            this.start = end;
            this.street = street;
        }

        public bool Equals(Edge edge)
        {
            return (start.Equals(edge.start) && end.Equals(edge.end) && street.Equals(edge.street));
        } 

        public double Length()
        {
            return Math.Sqrt(Math.Pow(end.x - start.x, 2) + Math.Pow(end.y - start.y, 2));
        }

    }



}