package pa9;

import java.util.*;

public class GraphList {

    private static class Edge implements Comparable<Edge> {
        int source;
        int destination;
        int weight;

        public Edge(int source, int destination, int weight) {
            this.source = source;
            this.destination = destination;
            this.weight = weight;
        }

        @Override
        public int compareTo(Edge other) {
            return Integer.compare(this.weight, other.weight);
        }
    }

    private int vertices; // Number of vertices
    private List<List<Edge>> adjacencyList; // Adjacency list for the graph
    private List<Edge> edges; // List of all edges (for Kruskal's algorithm)

    // Constructor
    public GraphList(int vertices) {
        this.vertices = vertices;
        adjacencyList = new ArrayList<>();
        edges = new ArrayList<>(); // Initialize edge list
        for (int i = 0; i < vertices; i++) {
            adjacencyList.add(new ArrayList<>());
        }
    }

    // Add a weighted edge to the graph
    public void addWeightedEdge(int source, int destination, int weight) {
        adjacencyList.get(source).add(new Edge(source, destination, weight));
        adjacencyList.get(destination).add(new Edge(destination, source, weight)); // For undirected graph
        edges.add(new Edge(source, destination, weight)); // Add to edge list for Kruskal's
    }

    // Bellman-Ford algorithm to find the shortest path
    public int[] shortestPath(int source) {
        int[] distances = new int[vertices];
        Arrays.fill(distances, Integer.MAX_VALUE);
        distances[source] = 0;

        // Relax edges |V| - 1 times
        for (int i = 1; i <= vertices - 1; i++) {
            for (Edge edge : edges) {
                int u = edge.source;
                int v = edge.destination;
                int weight = edge.weight;

                if (distances[u] != Integer.MAX_VALUE && distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                }
            }
        }

        // Check for negative-weight cycles
        for (Edge edge : edges) {
            int u = edge.source;
            int v = edge.destination;
            int weight = edge.weight;

            if (distances[u] != Integer.MAX_VALUE && distances[u] + weight < distances[v]) {
                return null; 
            }
        }

        return distances;
    }

    // Kruskal's algorithm for Minimum Spanning Tree
    public int[] minimumSpanningTree() {
        // Sort edges by weight
        edges.sort(Edge::compareTo);

        // Initialize each vertex to be its own component
        int[] component = new int[vertices];
        for (int i = 0; i < vertices; i++) {
            component[i] = i;
        }

        int mstWeight = 0;
        List<Edge> mstEdges = new ArrayList<>();

        for (Edge edge : edges) {
            int u = edge.source;
            int v = edge.destination;

            // If u and v are in different components, add the edge to the MST
            if (component[u] != component[v]) {
                mstEdges.add(edge);
                mstWeight += edge.weight;

                // Update components: merge component of v into component of u
                int oldComponent = component[v];
                for (int i = 0; i < vertices; i++) {
                    if (component[i] == oldComponent) {
                        component[i] = component[u];
                    }
                }

                // Stop if we've added enough edges
                if (mstEdges.size() == vertices - 1) {
                    break;
                }
            }
        }

        // If we don't have enough edges, the graph is disconnected
        if (mstEdges.size() != vertices - 1) {
            return null;
        }

        // Print the MST
        for (Edge e : mstEdges) {
            System.out.println("Edge: " + e.source + " - " + e.destination + ", Weight: " + e.weight);
        }
        System.out.println("Total MST Weight: " + mstWeight);

        return new int[]{mstWeight};
    }

    // Prim's algorithm to find the minimum spanning tree
    public int[] minimumSpanningTreePrim() {
        boolean[] visited = new boolean[vertices];
        PriorityQueue<Edge> priorityQueue = new PriorityQueue<>();
        List<Edge> mstEdges = new ArrayList<>();

        int mstWeight = 0;

        // Start from vertex 0
        visited[0] = true;
        priorityQueue.addAll(adjacencyList.get(0));

        while (!priorityQueue.isEmpty() && mstEdges.size() < vertices - 1) {
            Edge edge = priorityQueue.poll();

            // If the destination vertex is already visited, skip
            if (visited[edge.destination]) {
                continue;
            }

            // Add the edge to the MST
            mstEdges.add(edge);
            mstWeight += edge.weight;
            visited[edge.destination] = true;

            // Add all edges of the newly visited vertex to the priority queue
            for (Edge adjacentEdge : adjacencyList.get(edge.destination)) {
                if (!visited[adjacentEdge.destination]) {
                    priorityQueue.add(adjacentEdge);
                }
            }
        }

        // If we don't have enough edges, the graph is disconnected
        if (mstEdges.size() != vertices - 1) {
            return null;
        }

        // Print the MST
        for (Edge e : mstEdges) {
            System.out.println("Edge: " + e.source + " - " + e.destination + ", Weight: " + e.weight);
        }
        System.out.println("Total MST Weight: " + mstWeight);

        return new int[]{mstWeight};
    }

    // Main method for testing
    public static void main(String[] args) {
    }
}
