//Comparative analysis of A* algorithm and Dijkstra Algorithm to determine execution speed

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <ctime>
#include <limits>
#include <algorithm>

struct Point {
    int x, y;
    Point(int _x, int _y) : x(_x), y(_y) {}
};

struct Cell {
    int x, y;
    bool obstacle;
    Cell(int _x, int _y, bool _obstacle = false) : x(_x), y(_y), obstacle(_obstacle) {}
};

struct Node {
    Point point;
    int gScore;
    int fScore;
    Node(Point _point, int _gScore, int _fScore) : point(_point), gScore(_gScore), fScore(_fScore) {}
    bool operator<(const Node& other) const {
        return fScore > other.fScore; 
    }
};

bool isValid(int x, int y, int width, int height) {
    return x >= 0 && x < width && y >= 0 && y < height;
}

int heuristic(const Point& a, const Point& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<Point> reconstructPath(std::vector<std::vector<Point>>& cameFrom, Point current) {
    std::vector<Point> path;
    while (cameFrom[current.x][current.y].x != -1 && cameFrom[current.x][current.y].y != -1) {
        path.push_back(current);
        current = cameFrom[current.x][current.y];
    }
    path.push_back(current);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Point> AStar(const Point& start, const Point& goal, const std::vector<Cell>& obstacles, int width, int height) {
    std::vector<std::vector<Point>> cameFrom(width, std::vector<Point>(height, Point(-1, -1)));
    std::vector<std::vector<int>> gScore(width, std::vector<int>(height, std::numeric_limits<int>::max()));
    std::vector<std::vector<int>> fScore(width, std::vector<int>(height, std::numeric_limits<int>::max()));

    gScore[start.x][start.y] = 0;
    fScore[start.x][start.y] = heuristic(start, goal);

    std::priority_queue<Node> openSet;
    openSet.emplace(start, 0, heuristic(start, goal));

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current.point.x == goal.x && current.point.y == goal.y) {
            return reconstructPath(cameFrom, goal);
        }

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int neighborX = current.point.x + dx;
                int neighborY = current.point.y + dy;

                if (!isValid(neighborX, neighborY, width, height)) continue;

                int tentativeGScore = current.gScore + 1; 

                if (tentativeGScore < gScore[neighborX][neighborY]) {
                    Point neighbor(neighborX, neighborY);
                    if (std::find_if(obstacles.begin(), obstacles.end(), [&](const Cell& cell) {
                        return cell.x == neighborX && cell.y == neighborY && cell.obstacle;
                    }) != obstacles.end()) {
                        continue; 
                    }

                    cameFrom[neighborX][neighborY] = current.point;
                    gScore[neighborX][neighborY] = tentativeGScore;
                    fScore[neighborX][neighborY] = tentativeGScore + heuristic(neighbor, goal);
                    openSet.emplace(neighbor, tentativeGScore, fScore[neighborX][neighborY]);
                }
            }
        }
    }

    return std::vector<Point>(); 
}

std::vector<Point> dijkstra(const Point& start, const Point& goal, const std::vector<Cell>& obstacles, int width, int height) {
    std::vector<std::vector<Point>> cameFrom(width, std::vector<Point>(height, Point(-1, -1)));
    std::vector<std::vector<int>> gScore(width, std::vector<int>(height, std::numeric_limits<int>::max()));

    gScore[start.x][start.y] = 0;

    std::priority_queue<Node> openSet;
    openSet.emplace(start, 0, 0);

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current.point.x == goal.x && current.point.y == goal.y) {
            return reconstructPath(cameFrom, goal);
        }

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int neighborX = current.point.x + dx;
                int neighborY = current.point.y + dy;

                if (!isValid(neighborX, neighborY, width, height)) continue;

                int tentativeGScore = current.gScore + 1; 

                if (tentativeGScore < gScore[neighborX][neighborY]) {
                    Point neighbor(neighborX, neighborY);
                    if (std::find_if(obstacles.begin(), obstacles.end(), [&](const Cell& cell) {
                        return cell.x == neighborX && cell.y == neighborY && cell.obstacle;
                    }) != obstacles.end()) {
                        continue; 
                    }

                    cameFrom[neighborX][neighborY] = current.point;
                    gScore[neighborX][neighborY] = tentativeGScore;
                    openSet.emplace(neighbor, tentativeGScore, 0); 
                }
            }
        }
    }

    return std::vector<Point>(); 
}

int main() {
    std::vector<Cell> obstacles = {
        Cell(5, 5), Cell(5, 6), Cell(5, 7), Cell(5, 8), Cell(5, 9), Cell(5, 10), Cell(5, 11),
        Cell(15, 15), Cell(16, 15), Cell(17, 15), Cell(18, 15), Cell(19, 15), Cell(20, 15), Cell(21, 15),
        Cell(25, 25), Cell(25, 26), Cell(25, 27), Cell(25, 28), Cell(25, 29), Cell(25, 30), Cell(25, 31),
        Cell(35, 35), Cell(36, 35), Cell(37, 35), Cell(38, 35), Cell(39, 35), Cell(40, 35), Cell(41, 35),
    };
  
    Point start(0, 0);
    Point goal(99, 99);

    std::clock_t start_time_astar = std::clock();
    std::vector<Point> path_astar = AStar(start, goal, obstacles, 100, 100);
    double duration_astar = (double)(std::clock() - start_time_astar) / (double)CLOCKS_PER_SEC;

    std::clock_t start_time_dijkstra = std::clock();
    std::vector<Point> path_dijkstra = dijkstra(start, goal, obstacles, 100, 100);
    double duration_dijkstra = (double)(std::clock() - start_time_dijkstra) / (double)CLOCKS_PER_SEC;

    std::cout << "A* Path: ";
    if (!path_astar.empty()) {
        for (const auto& point : path_astar) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
    } else {
        std::cout << "No path found.";
    }
    std::cout << std::endl;
    std::cout << std::endl;

    std::cout << "Dijkstra Path: ";
    if (!path_dijkstra.empty()) {
        for (const auto& point : path_dijkstra) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
    } else {
        std::cout << "No path found.";
    }
    std::cout << std::endl;
    std::cout << std::endl;

    std::cout << "A* Execution Time: " << duration_astar << " seconds" << std::endl;
    std::cout << std::endl;
    std::cout << "Dijkstra Execution Time: " << duration_dijkstra << " seconds" << std::endl;

    return 0;
}
