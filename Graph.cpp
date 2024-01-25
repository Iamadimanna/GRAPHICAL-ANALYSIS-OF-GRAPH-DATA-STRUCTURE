#include<bits/stdc++.h>
using namespace std;

bool isBipartiteBFS(vector<vector<int>>& adj, int src, vector<int>& color) {
    // -1: not colored, 0: color 0, 1: color 1
    color[src] = 0;

    queue<int> q;
    q.push(src);

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int v : adj[u]) {
            if (color[v] == -1) {
                // Assign the opposite color to the adjacent vertex
                color[v] = 1 - color[u];
                q.push(v);
            } else if (color[v] == color[u]) {
                // If the adjacent vertices have the same color, the graph is not bipartite
                return false;
            }
        }
    }

    return true;
}
void DFSUtilly(vector<vector<int>>& adj, int v, vector<bool>& visited, vector<int>& component) {
    // Mark the current vertex as visited
    visited[v] = true;

    // Add the current vertex to the connected component
    component.push_back(v);

    // Recur for all the adjacent vertices not yet visited
    for (int u : adj[v]) {
        if (!visited[u]) {
            DFSUtilly(adj, u, visited, component);
        }
    }
}
void DFSUtil(vector<vector<int>>& adj, int v, vector<bool>& visited) {
    // Mark the current vertex as visited
    visited[v] = true;
    
    // Process the current vertex (You can print it, perform some operation, etc.)
    cout << v << " ";

    // Recur for all the adjacent vertices not yet visited
    for (int u : adj[v]) {
        if (!visited[u]) {
            DFSUtil(adj, u, visited);
        }
    }
}
void DFS(vector<vector<int>>& adj, int v, vector<bool>& visited){
     DFSUtil(adj, v, visited);
}
void BFS(vector<vector<int>>& adj, int v, vector<bool>& visited){
      // Create a queue for BFS
    queue<int> q;

    // Mark the current vertex as visited and enqueue it
    visited[v] = true;
    q.push(v);

    cout << "BFS Traversal:" << endl;

    while (!q.empty()) {
        // Dequeue a vertex from the queue and print it
        v = q.front();
        cout << v << " ";
        q.pop();

        // Enqueue all adjacent vertices of the dequeued vertex that are not yet visited
        for(int u : adj[v]) {
            if (!visited[u]) {
                visited[u] = true;
                q.push(u);
            }
        }
    }

    cout << endl;
}
void findConnectedComponents(vector<vector<int>>& adj, int N){
      vector<bool> visited(N, false);

    cout << "Connected Components:" << endl;

    for (int i = 0; i < N; ++i) {
        if (!visited[i]) {
            vector<int> component;
            DFSUtilly(adj, i, visited, component);

            // Print the connected component
            for (int vertex : component) {
                cout << vertex << " ";
            }
            cout << endl;
        }
    }
}
bool isBipartite(vector<vector<int>>& adj, int N){
     // -1: not colored, 0: color 0, 1: color 1
    vector<int> color(N, -1);

    for (int i = 0; i < N; ++i) {
        if (color[i] == -1) {
            if (!isBipartiteBFS(adj, i, color)) {
                return false;
            }
        }
    }

    return true;
}
bool isCyclicUtil(vector<vector<int>>& adj, int v, vector<bool>& visited, int parent){
       visited[v] = true;

    for (int u : adj[v]) {
        if (!visited[u]) {
            // If an adjacent vertex is not visited, recur for that vertex
            if (isCyclicUtil(adj, u, visited, v)) {
                return true;
            }
        } else if (u != parent) {
            // If the adjacent vertex is visited and not the parent of the current vertex,
            // then there is a cycle in the graph
            return true;
        }
    }

    return false;
}
bool isCyclic(vector<vector<int>>& adj, int N){
     vector<bool> visited(N, false);

    for (int i = 0; i < N; ++i) {
        if (!visited[i]) {
            // If the DFS traversal from any unvisited vertex finds a cycle, return true
            if (isCyclicUtil(adj, i, visited, -1)) {
                return true;
            }
        }
    }

    return false;
}
void topologicalSortUtil(vector<vector<int>>& adj, int v, vector<bool>& visited, stack<int>& s){
      // Mark the current vertex as visited
    visited[v] = true;

    // Recur for all the adjacent vertices not yet visited
    for (int u : adj[v]) {
        if (!visited[u]) {
            topologicalSortUtil(adj, u, visited, s);
        }
    }

    // Push the current vertex onto the stack
    s.push(v);
}
void topologicalSort(vector<vector<int>>& adj, int N){
    stack<int> s;
    vector<bool> visited(N, false);

    cout << "Topological Sorting:" << endl;

    for (int i = 0; i < N; ++i) {
        if (!visited[i]) {
            topologicalSortUtil(adj, i, visited, s);
        }
    }

    while (!s.empty()) {
        cout << s.top() << " ";
        s.pop();
    }

    cout << endl;
}
void dijkstra(vector<vector<pair<int, int>>>& adj, int src, int N) {
    // Priority queue to store vertices and their distances
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Vector to store distances from the source to all vertices
    vector<int> dist(N, INT_MAX);

    // Enqueue the source vertex with distance 0
    pq.push({0, src});
    dist[src] = 0;

    while (!pq.empty()) {
        // Extract the vertex with the minimum distance
        int u = pq.top().second;
        pq.pop();

        // Update the distances of the adjacent vertices
        for (pair<int, int> neighbor : adj[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                // Relaxation step
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    // Print the distances from the source vertex to all other vertices
    cout << "Shortest distances from vertex " << src << " to all other vertices:" << endl;
    for (int i = 0; i < N; ++i) {
        cout << "To vertex " << i << ": " << (dist[i] == INT_MAX ? "INF" : to_string(dist[i])) << endl;
    }
}
void bellmanFord(vector<vector<pair<int, int>>>& adj, int src, int N) {
    // Vector to store distances from the source to all vertices
    vector<int> dist(N, INT_MAX);

    // Initialize distance to the source vertex as 0
    dist[src] = 0;

    // Relax edges |V| - 1 times
    for (int i = 0; i < N - 1; ++i) {
        for (int u = 0; u < N; ++u) {
            for (pair<int, int> neighbor : adj[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                    // Relaxation step
                    dist[v] = dist[u] + weight;
                }
            }
        }
    }

    // Check for negative cycles
    for (int u = 0; u < N; ++u) {
        for (pair<int, int> neighbor : adj[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                cout << "Graph contains a negative cycle. Bellman-Ford cannot find the shortest paths." << endl;
                return;
            }
        }
    }

    // Print the distances from the source vertex to all other vertices
    cout << "Shortest distances from vertex " << src << " to all other vertices:" << endl;
    for (int i = 0; i < N; ++i) {
        cout << "To vertex " << i << ": " << (dist[i] == INT_MAX ? "INF" : to_string(dist[i])) << endl;
    }
}
void floydWarshall(vector<vector<int>>& graph, int N) {
    // Initialize the distance matrix with the given graph
    vector<vector<int>> dist(graph);

    // Update the distance matrix by considering all vertices as intermediate vertices
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                // If vertex k is on the shortest path from i to j, then update the distance
                if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX && dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }

    // Print the shortest distances between all pairs of vertices
    cout << "Shortest distances between all pairs of vertices:" << endl;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            cout << "From vertex " << i << " to vertex " << j << ": ";
            if (dist[i][j] == INT_MAX) {
                cout << "INF";
            } else {
                cout << dist[i][j];
            }
            cout << endl;
        }
    }
}
void primMST(vector<vector<pair<int, int>>>& adj, int N) {
    // Priority queue to store edges and their weights
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Vector to keep track of visited vertices
    vector<bool> visited(N, false);

    // Start with the first vertex
    int src = 0;
    visited[src] = true;

    // Enqueue all edges from the source vertex
    for (pair<int, int> neighbor : adj[src]) {
        int v = neighbor.first;
        int weight = neighbor.second;
        pq.push({weight, v});
    }

    cout << "Minimum Spanning Tree (Prim's Algorithm):" << endl;

    while (!pq.empty()) {
        // Extract the minimum edge
        int weight = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // If the vertex is not visited, include it in the MST
        if (!visited[u]) {
            cout << "Edge: " << src << " - " << u << " Weight: " << weight << endl;
            visited[u] = true;

            // Enqueue all edges from the newly included vertex
            for (pair<int, int> neighbor : adj[u]) {
                int v = neighbor.first;
                int w = neighbor.second;
                pq.push({w, v});
            }
        }
    }
}
void bridgeDFS(int u, int parent, vector<int>& disc, vector<int>& low, vector<bool>& visited, vector<vector<int>>& adj) {
    static int time = 0;

    disc[u] = low[u] = ++time;
    visited[u] = true;

    for (int v : adj[u]) {
        if (!visited[v]) {
            bridgeDFS(v, u, disc, low, visited, adj);

            // Update low value after the recursive call
            low[u] = min(low[u], low[v]);

            // Check for bridge
            if (low[v] > disc[u]) {
                cout << "Bridge: " << u << " - " << v << endl;
            }
        } else if (v != parent) {
            // Update low value if the adjacent vertex is already visited
            low[u] = min(low[u], disc[v]);
        }
    }
}
void findBridges(vector<vector<int>>& adj, int N){
      vector<int> disc(N, -1);   // Discovery time
    vector<int> low(N, -1);    // Low value
    vector<bool> visited(N, false);

    cout << "Bridges in the graph:" << endl;

    for (int i = 0; i < N; ++i) {
        if (!visited[i]) {
            bridgeDFS(i, -1, disc, low, visited, adj);
        }
    }
}
void findArticulationPointsDFS(int u, vector<vector<int>>& adj, vector<int>& disc, vector<int>& low, vector<int>& parent, vector<bool>& articulationPoints) {
    static int time = 0; // Time of discovery for each vertex

    disc[u] = low[u] = ++time; // Initialize discovery time and low value for the current vertex

    int children = 0; // Count of children in DFS tree

    for (int v : adj[u]) {
        if (disc[v] == -1) {
            children++;
            parent[v] = u;

            findArticulationPointsDFS(v, adj, disc, low, parent, articulationPoints);

            // Update low value for the current vertex based on its children
            low[u] = min(low[u], low[v]);

            // Check if the current vertex is an articulation point
            if ((parent[u] == -1 && children > 1) || (parent[u] != -1 && low[v] >= disc[u])) {
                articulationPoints[u] = true;
            }
        } else if (v != parent[u]) {
            // Update low value for the current vertex based on the back edge
            low[u] = min(low[u], disc[v]);
        }
    }
}
void findArticulationPoints(vector<vector<int>>& adj, int N){
      vector<int> disc(N, -1); // Discovery time of each vertex
    vector<int> low(N, -1);  // Low value of each vertex
    vector<int> parent(N, -1); // Parent of each vertex in DFS tree
    vector<bool> articulationPoints(N, false); // To mark articulation points

    cout << "Articulation Points in the Graph:" << endl;

    for (int i = 0; i < N; ++i) {
        if (disc[i] == -1) {
            findArticulationPointsDFS(i, adj, disc, low, parent, articulationPoints);
        }
    }

    for (int i = 0; i < N; ++i) {
        if (articulationPoints[i]) {
            cout << i << " ";
        }
    }

    cout << endl;
}

int main() {
    int N, M;
    cout << "Enter the number of vertices (N): ";
    cin >> N;
    cout << "Enter the number of edges (M): ";
    cin >> M;

    // Input graph as a vector of adjacency lists
    vector<vector<int>> adj(N);
    vector<vector<pair<int, int>>> weightedAdj(N);

    cout << "Enter the edges (u v) for the graph:" << endl;
    for (int i = 0; i < M; ++i) {
        int u, v, w;
        cin >> u >> v;
        cout << "Enter the weight of the edge between " << u << " and " << v << " (for unweighted graph, enter 1): ";
        cin >> w;

        adj[u].push_back(v);
        weightedAdj[u].push_back({v, w});
        // For undirected graph
        adj[v].push_back(u);
        weightedAdj[v].push_back({u, w});
    }

    int choice;
    do {
        cout << "\nGraph Algorithms Menu:" << endl;
        cout << "1. DFS\n2. BFS\n3. Connected Components\n4. Bipartite Graph\n"
             << "5. Cycle Detection\n6. Topological Sorting\n7. Dijkstra's Shortest Path\n"
             << "8. Bellman-Ford Shortest Path\n9. Floyd-Warshall All Shortest Paths\n"
             << "10. Minimum Spanning Tree (Prim's Algorithm)\n11. Bridges in Graph\n"
             << "12. Articulation Points\n0. Exit\n";
        cout << "Enter your choice: ";
        cin >> choice;

        switch (choice) {
            case 1: {
                vector<bool> visited(N, false);
                cout << "DFS Traversal:" << endl;
                for (int i = 0; i < N; ++i) {
                    if (!visited[i]) {
                        DFS(adj, i, visited);
                    }
                }
                break;
            }
            case 2: {
                vector<bool> visited(N, false);
                cout << "BFS Traversal:" << endl;
                for (int i = 0; i < N; ++i) {
                    if (!visited[i]) {
                        BFS(adj, i, visited);
                    }
                }
                break;
            }
            case 3:
                findConnectedComponents(adj, N);
                break;
            case 4:
                cout << (isBipartite(adj, N) ? "Graph is Bipartite." : "Graph is not Bipartite.") << endl;
                break;
            case 5:
                cout << (isCyclic(adj, N) ? "Graph contains a cycle." : "Graph is acyclic.") << endl;
                break;
            case 6: {
                stack<int> s;
                vector<bool> visited(N, false);
                cout << "Topological Sorting:" << endl;
                for (int i = 0; i < N; ++i) {
                    if (!visited[i]) {
                        topologicalSortUtil(adj, i, visited, s);
                    }
                }
                while (!s.empty()) {
                    cout << s.top() << " ";
                    s.pop();
                }
                cout << endl;
                break;
            }
            case 7: {
                int src;
                cout << "Enter the source vertex for Dijkstra's Shortest Path: ";
                cin >> src;
                dijkstra(weightedAdj, src, N);
                break;
            }
            case 8: {
                int src;
                cout << "Enter the source vertex for Bellman-Ford Shortest Path: ";
                cin >> src;
                bellmanFord(weightedAdj, src, N);
                break;
            }
            case 9:
                floydWarshall(adj, N);
                break;
            case 10:
                primMST(weightedAdj, N);
                break;
            case 11:
                findBridges(adj, N);
                break;
            case 12:
                findArticulationPoints(adj, N);
                break;
            case 0:
                cout << "Exiting the program." << endl;
                break;
            default:
                cout << "Invalid choice. Please enter a valid option." << endl;
        }
    } while (choice != 0);

    return 0;
}

// Implementations of the graph algorithms...
// (DFS, BFS, Connected Components, Bipartite Graph, Cycle Detection, Topological Sorting,
// Dijkstra's Shortest Path, Bellman-Ford Shortest Path, Floyd-Warshall All Shortest Paths,
// Minimum Spanning Tree, Bridges in Graph, Articulation Points)
// ...


