#ifndef MAZE_H
#define MAZE_H

#include <iostream>
#include <map>
#include <vector>
#include <fstream>
#include <algorithm>

using namespace std;

struct Point
{
    int x;
    int y;

    bool operator==(const Point other) const;

    Point operator+(const Point other) const;

    bool operator<(const Point other) const;
};

class Maze
{
private:
    int rows;
    int cols;

    /// @brief Representation of maze.
    vector <string> data;

    /// @brief Returns true if the point is inside.
    /// @param point
    /// @return bool
    bool is_inside(const Point point);

    /// @brief Finds position of start.
    /// @details Result is dynamically allocated.
    /// @return Position of start.
    Point *find_start();

    /// @brief Finds position of goal.
    /// @details Result is dynamically allocated.
    /// @return Position of goal.
    Point *find_goal();

    /// @brief Traces path to it's beggining and writes "Maze::path_to_goal_"
    /// @param came_from Map for reverse path construction.
    /// @param head Head of path.
    /// @return void
    void reconstruct_path(map <Point, Point> &came_from, const Point current);

    /// @brief A* path finding algorithm.
    /// @param start Position of start.
    /// @param goal Position of goal.
    /// @return void
    void astar(const Point start, const Point goal);

public:
    /// @brief Character that represents empty way in maze.
    static char empty_path_char;
    /// @brief Character that represents empty_wall in maze.
    static char wall_char;
    /// @brief Character that represents start in maze.
    static char start_char;
    /// @brief Character that represents goal in maze.
    static char goal_char;
    /// @brief Character that represents path to goal in maze.
    static char path_char;

    /// @brief Reads maze from text file.
    /// @param filepath
    /// @return Instance of class Maze.
    static Maze from_file(const string filepath);

    /// @brief Writes maze to text text file.
    /// @param filepath
    /// @return void
    void save_to(const string filepath);

    /// @brief Prints maze to console.
    /// @return void
    void print();

    /// @brief Solves maze in-place.
    /// @details Using A* search algorithm.
    /// @return void
    void solve();
};

#endif // MAZE_H