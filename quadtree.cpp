#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <limits>
#include <time.h>

//Implementation of Path Finding Algorithm using QuadTrees

struct Point {
    int x, y;
    Point(int _x, int _y) : x(_x), y(_y) {}
};

struct Cell {
    int x, y;
    bool obstacle;
    Cell(int _x, int _y, bool _obstacle = false) : x(_x), y(_y), obstacle(_obstacle) {}
};

struct QuadTreeNode {
    QuadTreeNode *children[4];
    std::vector<Cell*> cells;
    int x, y, size;

    QuadTreeNode(int _x, int _y, int _size) : x(_x), y(_y), size(_size) {
        for (int i = 0; i < 4; ++i) children[i] = nullptr;
    }

    ~QuadTreeNode() {
        for (int i = 0; i < 4; ++i) {
            delete children[i];
        }
    }

    bool isLeaf() const {
        return children[0] == nullptr;
    }
};

class QuadTree {
private:
    QuadTreeNode *root;
    int maxNodeSize;

    void insert(Cell *cell, QuadTreeNode *node) {
        if (node->isLeaf()) {
            node->cells.push_back(cell);
            if (node->cells.size() > maxNodeSize && node->size > 1) {
                split(node);
            }
        } else {
            int index = getIndex(cell, node);
            if (index != -1) {
                insert(cell, node->children[index]);
            } else {
                node->cells.push_back(cell);
            }
        }
    }

    int getIndex(Cell *cell, QuadTreeNode *node) {
        int index = -1;
        int verticalMidpoint = node->x + (node->size / 2);
        int horizontalMidpoint = node->y + (node->size / 2);

        bool topQuadrant = cell->y < horizontalMidpoint && cell->y + 1 >= horizontalMidpoint;
        bool bottomQuadrant = cell->y >= horizontalMidpoint && cell->y + 1 < horizontalMidpoint;
        bool leftQuadrant = cell->x < verticalMidpoint && cell->x + 1 >= verticalMidpoint;
        bool rightQuadrant = cell->x >= verticalMidpoint && cell->x + 1 < verticalMidpoint;

        if (leftQuadrant) {
            if (topQuadrant) index = 1;
            else if (bottomQuadrant) index = 2;
        } else if (rightQuadrant) {
            if (topQuadrant) index = 0;
            else if (bottomQuadrant) index = 3;
        }

        return index;
    }

    void split(QuadTreeNode *node) {
        int subSize = node->size / 2;
        node->children[0] = new QuadTreeNode(node->x + subSize, node->y, subSize);
        node->children[1] = new QuadTreeNode(node->x, node->y, subSize);
        node->children[2] = new QuadTreeNode(node->x, node->y + subSize, subSize);
        node->children[3] = new QuadTreeNode(node->x + subSize, node->y + subSize, subSize);

        for (auto it = node->cells.begin(); it != node->cells.end();) {
            Cell *cell = *it;
            int index = getIndex(cell, node);
            if (index != -1) {
                node->children[index]->cells.push_back(cell);
                it = node->cells.erase(it);
            } else {
                ++it;
            }
        }
    }

public:
    QuadTree(int size, int maxNodeSize) : maxNodeSize(maxNodeSize) {
        root = new QuadTreeNode(0, 0, size);
    }

    ~QuadTree() {
        delete root;
    }

    void insert(Cell *cell) {
        insert(cell, root);
    }

    std::vector<Cell*> queryRange(int x, int y, int radius) {
        std::vector<Cell*> result;
        queryRange(root, x, y, radius, result);
        return result;
    }

    void queryRange(QuadTreeNode *node, int x, int y, int radius, std::vector<Cell*> &result) {
        if (node == nullptr) return;

        if (node->isLeaf()) {
            for (Cell *cell : node->cells) {
                if (std::abs(cell->x - x) <= radius && std::abs(cell->y - y) <= radius) {
                    result.push_back(cell);
                }
            }
        } else {
            int index = getIndex(new Cell(x, y), node);
            if (index != -1) {
                queryRange(node->children[index], x, y, radius, result);
            }
            for (int i = 0; i < 4; ++i) {
                if (i != index) {
                    if (node->children[i]->x + node->children[i]->size >= x - radius &&
                        node->children[i]->y + node->children[i]->size >= y - radius &&
                        node->children[i]->x <= x + radius &&
                        node->children[i]->y <= y + radius) {
                        queryRange(node->children[i], x, y, radius, result);
                    }
                }
            }
        }
    }
};

struct Node {
    Point point;
    int gScore;
    int fScore;
    clock_t t;

    Node(Point _point, int _gScore, int _fScore) : point(_point), gScore(_gScore), fScore(_fScore) {}

    bool operator<(const Node& other) const {
        return fScore > other.fScore; // For min-heap
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

                int tentativeGScore = current.gScore + 1; // Assuming uniform cost

                if (tentativeGScore < gScore[neighborX][neighborY]) {
                    Point neighbor(neighborX, neighborY);
                    if (std::find_if(obstacles.begin(), obstacles.end(), [&](const Cell& cell) {
                        return cell.x == neighborX && cell.y == neighborY && cell.obstacle;
                    }) != obstacles.end()) {
                        continue; // Skip obstacles
                    }

                    cameFrom[neighborX][neighborY] = current.point;
                    gScore[neighborX][neighborY] = tentativeGScore;
                    fScore[neighborX][neighborY] = tentativeGScore + heuristic(neighbor, goal);
                    openSet.emplace(neighbor, tentativeGScore, fScore[neighborX][neighborY]);
                }
            }
        }
    }

    return std::vector<Point>(); //no path found
}

int main() {
    clock_t t;
    int width = 100;
    int height = 100;
    QuadTree quadTree(width, 4); 

    std::vector<Cell> obstacles = {
        Cell(20, 50, true),
        Cell(21, 50, true),
        Cell(22, 50, true),
        Cell(23, 50, true),
        Cell(24, 50, true),
        Cell(25, 50, true),
        Cell(50, 20, true),
        Cell(50, 21, true),
        Cell(50, 22, true),
        Cell(50, 23, true),
        Cell(50, 24, true),
        Cell(50, 50, true),
    };

    Point start(0, 0);
    Point goal(99, 99);
    
    t=clock();

    for (auto& obstacle : obstacles) {
        quadTree.insert(&obstacle);
    }

    std::vector<Point> path = AStar(start, goal, obstacles, width, height);
    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (const auto& point : path) {
            std::cout << " -> (" << point.x << "," << point.y << ")";
        }
    } else {
        std::cout << "No path found." << std::endl;
    }

    t=clock()-t;
    std::cout<<" time taken: "<<(double)t/(double)CLOCKS_PER_SEC;
    return 0;
}
