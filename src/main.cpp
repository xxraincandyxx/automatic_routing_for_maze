// main.cpp
//
#include "../include/direction.h"

// Class init
Direction dirt(0, 0);

void init(int rows, int cols) {
    dirt = Direction(rows, cols);
}

int* get_direction(int x, int y, const std::vector<int>& x_buffer, const std::vector<int>& y_buffer, const std::vector<int8_t>& v_buffer, int length) {
    // Initialization
    std::tuple<int, int> loc  = { x, y };
    std::tuple<int, int> dir  = { 0, 0 };
    std::tuple<int, int> dest = { 0, 0 };
 
    // Add current location to path
    dirt.path_mod({loc});
    /*std::cout << "Check Point 2\n" << std::endl;*/

    // Vision
    dest = dirt.visualize(x_buffer, y_buffer, v_buffer, length);
    
    // Find the nearest black block
    dir = dirt.nearest_bblock_path(loc);

    // Dest Rush
    if (dirt.toggle == true) {
        dir = dirt.path_pop(loc);

        // Debug
        // dirt.draw_maze();
        // std::cout << "Dest Rush" << std::endl;
        return dirt.redef(dir);
    }

    // Dest Check
    if (dest != std::make_tuple(0, 0)) {
        dir = dirt.destination(loc, dest);

        // Debug
        // dirt.draw_maze();
        // std::cout << "Dest Check" << std::endl;
        return dirt.redef(dir);
    }

    // Choose the direction
    dir = dirt.nearest_bblock_path(loc);
    if (dir != std::make_tuple(0, 0)) {
        // Debug
        // dirt.draw_maze();
        // std::cout << "Nearest Bblock" << std::endl;
        return dirt.redef(dir);
    }

    // Final Check
    dir = dirt.final_check(loc);
    if (dir != std::make_tuple(0, 0)) {
        // Debug
        // dirt.draw_maze();
        // std::cout << "Final Check" << std::endl;
        return dirt.redef(dir);
    }

    //
    // dirt.draw_maze();
    std::cerr << "Error: Null pointer dereferenced." << std::endl;
    return NULL;  // Error Reference
}

int main() {
    int rows = 22;
    int cols = 32;

    // Init maze
    init(rows, cols);

    std::vector<std::vector<int8_t>> maze = {
    {0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1},
    {0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
    {0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1},
    {0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
    {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1},
    {0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1},
    {0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1},
    {1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1},
    {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},
    {0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1},
    {0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
    {0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1},
    {1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1},
    {0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 2}
    };

    std::tuple<int, int> start = std::make_tuple(0, 0);
    std::tuple<int, int> goal = std::make_tuple(21, 31);
    std::tuple<int, int> current = start;
    int radius = 2;
    
    std::vector<std::tuple<int, int>> path;

    while (true) {
        int x_start = std::max(0, std::get<0>(current) - radius);
        int x_end = std::min(rows, std::get<0>(current) + radius + 1);
        int y_start = std::max(0, std::get<1>(current) - radius);
        int y_end = std::min(cols, std::get<1>(current) + radius + 1);

        /*std::cout << "Check Point 00: " << static_cast<int>(maze[0][0]) << std::endl;*/

        std::vector<int> x_buffer;
        std::vector<int> y_buffer;
        std::vector<int8_t> v_buffer;

        int length = 0;
        for (int i = x_start; i < x_end; ++i) {
            for (int j = y_start; j < y_end; ++j) {
                x_buffer.push_back(i);
                y_buffer.push_back(j);
                v_buffer.push_back(maze[i][j]);
                /*std::cout << "Check Point 0: " << i << ", " << j << "   " << static_cast<int>(maze[i][j]) << std::endl;*/
                length++;
            }
        }

        /*std::cout << std::endl
            << "###############################################################################################" 
            << std::endl << std::endl;*/

        int* move = get_direction(std::get<0>(current), std::get<1>(current), x_buffer, y_buffer, v_buffer, length);
        current = std::make_tuple(std::get<0>(current) + move[0], std::get<1>(current) + move[1]);

        // Debug
        path.push_back(current);

        // Reach the goal
        if (maze[std::get<0>(current)][std::get<1>(current)] == 2) {
            std::cout << "Final Location: (" << std::setw(2) << std::get<0>(current) << "," <<
                std::setw(3) << std::get<1>(current) << std::setw(2) << ")" << std::endl;
            free(move);

            // Debug
            for (const auto& elem : path) {
                int x = std::get<0>(elem);
                int y = std::get<1>(elem);
                std::cout << "Coordinates: (" << x << ", " << y << ")" << std::endl;
            }

            break;
        }

        free(move);
    }

    return 0;
}