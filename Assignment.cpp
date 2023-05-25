//Assignment


#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

int networkDelayTime(vector<vector<int>>& times, int n, int k) {
    const int INF = numeric_limits<int>::max();
    
    // Create adjacency list representation of the graph
    vector<vector<pair<int, int>>> graph(n + 1);
    for (const auto& time : times) {
        int source = time[0];
        int target = time[1];
        int weight = time[2];
        graph[source].push_back({target, weight});
    }
    
    // Initialize distance array with infinity
    vector<int> distance(n + 1, INF);
    distance[k] = 0;
    
    // Priority queue for Dijkstra's algorithm
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, k});
    
    while (!pq.empty()) {
        int currNode = pq.top().second;
        int currDist = pq.top().first;
        pq.pop();
        
        // Skip if the current distance is greater than the stored distance
        if (currDist > distance[currNode]) {
            continue;
        }
        
        // Explore neighbors of the current node
        for (const auto& neighbor : graph[currNode]) {
            int nextNode = neighbor.first;
            int weight = neighbor.second;
            
            // Calculate the new distance
            int newDist = currDist + weight;
            
            // If the new distance is smaller, update the distance array
            if (newDist < distance[nextNode]) {
                distance[nextNode] = newDist;
                pq.push({newDist, nextNode});
            }
        }
    }
    
    // Find the maximum distance in the distance array
    int maxDelay = 0;
    for (int i = 1; i <= n; i++) {
        if (distance[i] == INF) {
            return -1;  // Some nodes are unreachable
        }
        maxDelay = max(maxDelay, distance[i]);
    }
    
    return maxDelay;
}

int main() {
    vector<vector<int>> times = {{2, 1, 1}, {2, 3, 1}, {3, 4, 1}};
    int n = 4;
    int k = 2;
    
    int minTime = networkDelayTime(times, n, k);
    cout<< minTime << endl;
    
    return 0;
}
