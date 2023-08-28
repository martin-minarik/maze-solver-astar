#include "maze.h"

bool Point::operator==(const Point other) const
{
    return (x == other.x) && (y == other.y);
}

Point Point::operator+(const Point other) const
{
    return Point{x + other.x, y + other.y};
}

bool Point::operator<(const Point other) const
{
    return (x < other.x) || (x == other.x && y < other.y);
}

int manhattan_distance(const Point &pos_a, const Point &pos_b)
{
    return abs(pos_a.x - pos_b.x) + abs(pos_a.y - pos_b.y);
}

char Maze::empty_path_char = '0';
char Maze::wall_char = '1';
char Maze::start_char = '2';
char Maze::goal_char = '3';
char Maze::path_char = '4';

bool Maze::is_inside(const Point point)
{
    return (point.x >= 0) && (point.x < this->cols) &&
           (point.y >= 0) && (point.y < this->rows);
}

void Maze::reconstruct_path(map <Point, Point> &came_from, Point current)
{
    while (came_from.find(current) != came_from.end())
    {
        char &current_char = this->data[current.y][current.x];
        if (current_char == Maze::empty_path_char)
            current_char = Maze::path_char;

        current = came_from[current];
    }
}

void Maze::astar(const Point start, const Point goal)
{
    vector <Point> open_set = {start};
    map <Point, Point> came_from;
    map<Point, int> g_scores;
    map<Point, int> f_scores;

    g_scores[start] = 0;
    f_scores[start] = manhattan_distance(start, goal);

    while (!open_set.empty())
    {
        Point current = open_set[0];

        // Find point with lowest f score
        for (Point &node: open_set)
            if (f_scores[node] < f_scores[current])
                current = node;

        int current_g_score = g_scores[current];

        if (current == goal)
            reconstruct_path(came_from, current);

        open_set.erase(remove(open_set.begin(), open_set.end(), current), open_set.end());

        // Check points around current point
        for (auto &direction: {Point{-1, 0}, Point{1, 0},
                               Point{0, -1}, Point{0, 1}})
        {
            Point neighbour = current + direction;
            int neighbour_g_score = current_g_score + 1;

            // Check if the neighbour is out of bounds
            if (!is_inside(neighbour))
                continue;

            // Check if the neighbour is wall
            if (this->data[neighbour.y][neighbour.x] == Maze::wall_char)
                continue;

            // Check if the neighbour doesn't have it's g score in g_scores
            // or if you found better g score than it's previous one
            // (better g score means better path to the point than previous one)
            if ((g_scores.find(neighbour) == g_scores.end()) ||
                (neighbour_g_score < g_scores[neighbour]))
            {
                came_from[neighbour] = current;
                g_scores[neighbour] = neighbour_g_score;
                f_scores[neighbour] = neighbour_g_score + manhattan_distance(neighbour, goal);

                // Check if the neighbour isn't already in open_set
                if (find(open_set.begin(), open_set.end(), neighbour) == open_set.end())
                {
                    open_set.push_back(neighbour);
                }
            }
        }
    }
}

Maze Maze::from_file(const string filepath)
{
    string line;

    ifstream file_stream(filepath);
    if (!file_stream)
    {
        cout << "Error(opening input file)!" << endl;
        exit(EXIT_FAILURE);
    }

    Maze maze;

    file_stream >> maze.rows;
    file_stream >> maze.cols;

    while (file_stream >> line)
        maze.data.push_back(line);

    file_stream.close();

    return maze;
}

void Maze::save_to(const string filepath)
{
    ofstream file_stream(filepath);
    if (!file_stream)
    {
        cout << "Error(opening output file)!" << endl;
        exit(EXIT_FAILURE);
    }

    file_stream << this->rows << endl;
    file_stream << this->cols << endl;

    for (string &line: this->data)
        file_stream << line << endl;

    file_stream.close();
}

void Maze::print()
{
    for (string &line: this->data)
        cout << line << endl;
}

Point *Maze::find_start()
{
    for (int row = 0; row < this->rows; ++row)
        for (int col = 0; col < this->cols; ++col)
        {
            if (this->data[row][col] == Maze::start_char)
            {
                return (new Point{col, row});
            }
        }

    return nullptr;
}

Point *Maze::find_goal()
{
    for (int row = 0; row < this->rows; ++row)
        for (int col = 0; col < this->cols; ++col)
        {
            if (this->data[row][col] == Maze::goal_char)
            {
                return (new Point{col, row});
            }
        }

    return nullptr;
}

void Maze::solve()
{
    Point *start = this->find_start();
    Point *goal = this->find_goal();

    // Check if the both start and goal were found
    if (start && goal)
    {
        this->astar(*start, *goal);
    } else
    {
        cout << "Error(missing Start and Goal nodes)!" << endl;
        exit(EXIT_FAILURE);
    }

    delete start;
    delete goal;
}
