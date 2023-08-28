#include <iostream>
#include "maze.h"

using namespace std;

int main()
{
    Maze maze = Maze::from_file("maze.txt");

    maze.solve();

    maze.print();
    maze.save_to("result.txt");

    return 0;
}