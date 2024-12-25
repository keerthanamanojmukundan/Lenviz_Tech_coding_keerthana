// submitted by Keerthana Manoj
#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <climits>

using namespace std;

// Base class DroneOperation using polymorphism
class DroneOperation
{
public:
    virtual void execute() = 0;
    virtual ~DroneOperation() {}
};

class TakeOff : public DroneOperation
{
public:
    void execute() override
    {
        cout << "Drone is taking off..." << endl;
    }
};

class Survey : public DroneOperation
{
public:
    void execute() override
    {
        cout << "Drone is surveying the route..." << endl;
    }
};

class ReturnToHome : public DroneOperation
{
public:
    void execute() override
    {
        cout << "Drone is returning to home..." << endl;
    }
};

class Land : public DroneOperation
{
public:
    void execute() override
    {
        cout << "Drone is landing..." << endl;
    }
};

// MissionPlanning class using bidirectional search
class MissionPlanning
{
private:
    vector<vector<int>> graph;
    int nodes;
    mutex mtx;

    // Helper function for bidirectional search
    bool bfs(queue<int> &q, vector<bool> &visited, vector<int> &parent, unordered_map<int, bool> &intersect, bool forward)
    {
        if (q.empty())
            return false;

        int current = q.front();
        q.pop();

        for (int neighbor = 0; neighbor < nodes; ++neighbor)
        {
            if (graph[current][neighbor] != 0 && !visited[neighbor])
            {
                visited[neighbor] = true;
                parent[neighbor] = current;
                q.push(neighbor);

                lock_guard<mutex> lock(mtx);
                if (intersect.find(neighbor) != intersect.end())
                {
                    return true; // Found intersection
                }

                // Add the node to the intersect map for the other search to find
                intersect[neighbor] = forward;
            }
        }
        return false;
    }

public:
    MissionPlanning(int n) : nodes(n), graph(n, vector<int>(n, 0)) {}

    void addEdge(int u, int v, int weight)
    {
        graph[u][v] = weight;
        graph[v][u] = weight;
    }

    vector<int> bidirectionalSearch(int start, int goal)
    {
        queue<int> q1, q2;
        vector<bool> visited1(nodes, false), visited2(nodes, false);
        vector<int> parent1(nodes, -1), parent2(nodes, -1);
        unordered_map<int, bool> intersect;

        q1.push(start);
        visited1[start] = true;
        intersect[start] = true; // Mark start for forward search

        q2.push(goal);
        visited2[goal] = true;
        intersect[goal] = false; // Mark goal for backward search

        while (!q1.empty() && !q2.empty())
        {
            if (bfs(q1, visited1, parent1, intersect, true))
                break;
            if (bfs(q2, visited2, parent2, intersect, false))
                break;
        }

        // Reconstruct path
        vector<int> path;
        for (const auto &[node, direction] : intersect)
        {
            if (visited1[node] && visited2[node])
            {
                // Found intersection point
                int mid = node;
                int current = mid;

                // Build path from start to mid
                while (current != -1)
                {
                    path.insert(path.begin(), current);
                    current = parent1[current];
                }

                // Build path from mid to goal
                current = parent2[mid];
                while (current != -1)
                {
                    path.push_back(current);
                    current = parent2[current];
                }
                break;
            }
        }

        return path;
    }
};

// Main driver function
int main()
{
    // Initialize classes
    TakeOff takeoff;
    Survey survey;
    ReturnToHome returnHome;
    Land land;

    // Execute Drone Operations
    takeoff.execute();

    MissionPlanning planner(100);
    for (int i = 0; i < 99; ++i)
    {
        planner.addEdge(i, i + 1, 1);
    }
    planner.addEdge(0, 99, 5);

    vector<int> path = planner.bidirectionalSearch(0, 99); // finding shortest path

    if (!path.empty())
    {
        cout << "Optimal Path: ";
        for (size_t i = 0; i < path.size() - 1; ++i)
        {
            cout << path[i] << " -> ";
        }
        cout << path.back() << endl;
    }
    else
    {
        cout << "No path found between nodes" << endl;
    }

    survey.execute();
    returnHome.execute();
    land.execute();

    return 0;
}
