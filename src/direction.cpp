// direction.cpp
//
#include "../include/direction.h"

// Initialize the class
Direction::Direction(int row, int column) : rows(row), cols(column), toggle(false) {
    // Initialize a black maze
    maze.resize(rows, std::vector<int8_t>(cols, -1));
}

// Append sub path to the path
void Direction::path_mod(const std::vector<std::tuple<int, int>>& sub_path) {
    std::move(sub_path.begin(), sub_path.end(), std::back_inserter(path));
}

// Return the direction from path
std::tuple<int, int> Direction::path_pop(const std::tuple<int, int>& loc) {
    if (path_tmp.empty()) return std::make_tuple(0, 0);

    std::tuple<int, int> ret_path = std::get<1>(path_tmp.top());
    path_tmp.pop();

    return std::make_tuple(std::get<0>(ret_path) - std::get<0>(loc), std::get<1>(ret_path) - std::get<1>(loc));
}

// Clear the path
void Direction::path_clr() {
    while (!path_tmp.empty()) path_tmp.pop();
}

// Calculate the distance of two blocks
float Direction::heuristic(const std::tuple<int, int>& a, const std::tuple<int, int>& b) {
    return (float)std::sqrt(std::pow(std::get<0>(a) - std::get<0>(b), 2) + std::pow(std::get<1>(a) - std::get<1>(b), 2));
}

// Black block search
std::tuple<int, int> Direction::block_search(const std::tuple<int, int>& start, const std::tuple<int, int>& goal) {
    std::priority_queue<
        std::tuple<float, std::tuple<int, int>>,
        std::vector<std::tuple<float, std::tuple<int, int>>>,
        std::greater<>> open_set;

    open_set.push(std::make_tuple(0.0f, start));

    std::unordered_map<std::tuple<int, int>, std::tuple<int, int>, tuple_hash> came_from;
    std::unordered_map<std::tuple<int, int>, int, tuple_hash> g_score;
    g_score[start] = 0;
    std::unordered_map<std::tuple<int, int>, float, tuple_hash> f_score;
    f_score[start] = heuristic(start, goal);

    while (!open_set.empty()) {
        // Debug
        /*std::cout << "open_set: " << std::endl;
        Direction::print_priority_queue(open_set);*/

        std::tuple<int, int> current = std::get<1>(open_set.top());
        open_set.pop();

        if (current == goal) {
            // Clear path_tmp cache
            path_clr();
            
            // Debug
            /*std::cout << "Block Search 1: " << std::get<0>(current) << ", " << std::get<1>(current) << std::endl;*/

            int idx = 0;
            while (came_from.find(current) != came_from.end()) {
                path_tmp.emplace(idx, current);
                current = came_from[current];
                idx -= 1;
            }
            
            // Trigger the toggle
            toggle = false;
            
            return path_pop(start);
        }

        for (auto dir : dirs) {
            std::tuple<int, int> neighbor = {std::get<0>(current) + std::get<0>(dir), std::get<1>(current) + std::get<1>(dir)};

            // Check the boundry
            if (std::get<0>(neighbor) >= 0 && std::get<0>(neighbor) < rows &&
                std::get<1>(neighbor) >= 0 && std::get<1>(neighbor) < cols &&
                maze[std::get<0>(neighbor)][std::get<1>(neighbor)] != 1) {
                
                /* Check the corner wall
                * if the two corners are blocked, then continue to next direction
                */
                if (std::abs(std::get<0>(dir)) + std::abs(std::get<1>(dir)) == 2) {  // Get the diagonal position, that's when the manhatten distance is 2
                    std::tuple<int, int> corner1 = { std::get<0>(current) + std::get<0>(dir), std::get<1>(current) };
                    std::tuple<int, int> corner2 = { std::get<0>(current), std::get<1>(current) + std::get<1>(dir) };

                    // if both of the corner are not path, then continue
                    if (maze[std::get<0>(corner1)][std::get<1>(corner1)] != 0 &&
                        maze[std::get<0>(corner2)][std::get<1>(corner2)] != 0) {
                        // Debug
                        /*std::cout << "Corner Check: (" << std::get<0>(neighbor) << ", " << std::get<1>(neighbor)
                            << ") = " << static_cast<int>(maze[std::get<0>(neighbor)][std::get<1>(neighbor)]) << std::endl;*/

                        continue;
                    }
                }

                // Main
                int tentative_g_score = g_score[current] + 1;
                
                // Debug
                /*std::cout << "Block Search 3: " << std::get<0>(neighbor) << ", " << std::get<1>(neighbor) 
                    << ", " << static_cast<int>(maze[std::get<0>(neighbor)][std::get<1>(neighbor)]) << std::endl;*/

                if (maze[std::get<0>(neighbor)][std::get<1>(neighbor)] == -1) {
                    // Clear path_tmp cache
                    path_clr();
                    
                    // Debug
                    /*std::cout << "Block Search 2: " << std::get<0>(neighbor) << ", " << std::get<1>(neighbor) << std::endl;*/

                    int idx = 0;
                    while (came_from.find(current) != came_from.end()) {
                        path_tmp.emplace(idx, current);
                        current = came_from[current];
                        idx -= 1;
                    }
                    
                    // Trigger the toggle
                    toggle = false;
                    
                    return path_pop(start);
                }
                
                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal);
                    open_set.emplace(f_score[neighbor], neighbor);
                }
            }
        }
    }

    // Path not found
    return std::make_tuple(0, 0);
}

// Destination rush
std::tuple<int, int> Direction::destination(const std::tuple<int, int>& start, const std::tuple<int, int>& goal) {
    std::priority_queue<
        std::tuple<float, std::tuple<int, int>>,
        std::vector<std::tuple<float, std::tuple<int, int>>>,
        std::greater<>> open_set;

    open_set.push(std::make_tuple(0.0f, start));

    std::unordered_map<std::tuple<int, int>, std::tuple<int, int>, tuple_hash> came_from;
    std::unordered_map<std::tuple<int, int>, int, tuple_hash> g_score;
    g_score[start] = 0;
    std::unordered_map<std::tuple<int, int>, float, tuple_hash> f_score;
    f_score[start] = heuristic(start, goal);

    while (!open_set.empty()) {
        std::tuple<int, int> current = std::get<1>(open_set.top());
        open_set.pop();

        if (current == goal) {
            // Clear path_tmp cache
            path_clr();
            
            int idx = 0;
            while (came_from.find(current) != came_from.end()) {
                path_tmp.emplace(idx, current);
                current = came_from[current];
                idx -= 1;
            }
            
            // Trigger the toggle
            toggle = false;
            
            return path_pop(start);
        }

        for (auto dir : dirs) {
            std::tuple<int, int> neighbor = {std::get<0>(current) + std::get<0>(dir), std::get<1>(current) + std::get<1>(dir)};

            // Check the boundry
            if (std::get<0>(neighbor) >= 0 && std::get<0>(neighbor) < rows &&
                std::get<1>(neighbor) >= 0 && std::get<1>(neighbor) < cols &&
                (maze[std::get<0>(neighbor)][std::get<1>(neighbor)] == 0 ||
                 maze[std::get<0>(neighbor)][std::get<1>(neighbor)] == 2)) {
                
                /* Check the corner wall
                * if the two corners are blocked, then continue to next direction
                */
                if (std::abs(std::get<0>(dir)) + std::abs(std::get<1>(dir)) == 2) {  // Get the diagonal position, that's when the manhatten distance is 2
                    std::tuple<int, int> corner1 = { std::get<0>(current) + std::get<0>(dir), std::get<1>(current) };
                    std::tuple<int, int> corner2 = { std::get<0>(current), std::get<1>(current) + std::get<1>(dir) };

                    // if both of the corner are not path, then continue
                    if (maze[std::get<0>(corner1)][std::get<1>(corner1)] != 0 &&
                        maze[std::get<0>(corner2)][std::get<1>(corner2)] != 0) {
                        // Debug
                        /*std::cout << "Corner Check: (" << std::get<0>(neighbor) << ", " << std::get<1>(neighbor)
                            << ") = " << static_cast<int>(maze[std::get<0>(neighbor)][std::get<1>(neighbor)]) << std::endl;*/

                        continue;
                    }
                }

                // Main
                int tentative_g_score = g_score[current] + 1;
                
                if (maze[std::get<0>(neighbor)][std::get<1>(neighbor)] == -1) {
                    // Clear path_tmp cache
                    path_clr();
                    
                    int idx = 0;
                    while (came_from.find(current) != came_from.end()) {
                        path_tmp.emplace(idx, current);
                        current = came_from[current];
                        idx -= 1;
                    }
                    
                    // Trigger the toggle
                    toggle = false;
                    
                    return path_pop(start);
                }
                
                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal);
                    open_set.emplace(f_score[neighbor], neighbor);
                }
            }
        }
    }

    // Path not found
    return std::make_tuple(0, 0);
}

// Find the nearest black block
std::tuple<int, int> Direction::nearest_bblock_path(const std::tuple<int, int>& loc) {
    std::priority_queue<std::tuple<float, std::tuple<int, int>>, 
                        std::vector<std::tuple<float, std::tuple<int, int>>>, 
                        std::greater<>> block_set;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (maze[i][j] == -1) {
                block_set.emplace(sqrt(pow(i - std::get<0>(loc), 2) + pow(j - std::get<1>(loc), 2)), std::make_tuple(i, j));
            }
            if (maze[i][j] == 2) {
                auto direct = destination(loc, std::make_tuple(i, j));
                if (direct != std::make_tuple(0, 0)) {
                    // Debug
                    // std::cout << "Direct ret: " << std::get<0>(direct) << ", " << std::get<1>(direct) << std::endl;

                    return direct;
                }
            }
        }
    }


    std::tuple<int, int> block_loc;
    std::tuple<float, std::tuple<int, int>> tmp;
    while (!block_set.empty()) {
        std::tuple<int, int> block_loc = std::get<1>(block_set.top());
        block_set.pop();

        // Debug
        //std::cout << "Block loc: " << std::get<0>(block_loc) << ", " << std::get<1>(block_loc) << std::endl;

        auto direct = block_search(loc, block_loc);

        if (direct != std::make_tuple(0, 0)) {
            while (!block_set.empty()) block_set.pop();  // Clear the block set
            return direct;
        }
    }

    // Path not found
    return std::make_tuple(0, 0);
}

// Vision
std::tuple<int, int> Direction::visualize(const std::vector<int>& x_buffer, const std::vector<int>& y_buffer, const std::vector<int8_t>& v_buffer, int length) {
    std::tuple<int, int> dest = {0, 0};

    for (int i = 0; i < length; ++i) {
        // Read vision to the maze
        int x = x_buffer[i];
        int y = y_buffer[i];

        /*std::cout << "Check Point 3: " << x << ", " << y << "   " << static_cast<int>(v_buffer[i]) << std::endl;*/

        maze[x][y] = v_buffer[i];

        // Check destination
        if (v_buffer[i] == 2) {
            dest = std::make_tuple(x, y);
        }

        // Initialize vision
        vision.emplace_back(x, y);
    }

    return dest;
}

// Find the direction which can score the most black blocks
std::tuple<int, int> Direction::direct(std::tuple<int, int> loc) {
    // Variables init
    std::tuple<int, int> current = loc;
    int maxBlock = 0;
    std::tuple<int, int> move_dir = {0, 0};

    // Direction loop
    for (const auto& dir : dirs) {
        // Calculate the number of the black block to be discovered
        std::tuple<int, int> neighbor = {std::get<0>(current) + std::get<0>(dir), std::get<1>(current) + std::get<1>(dir)};

        if (!(0 <= std::get<0>(neighbor) && std::get<0>(neighbor) < rows 
            && 0 <= std::get<1>(neighbor) && std::get<1>(neighbor) < cols) 
            || maze[std::get<0>(neighbor)][std::get<1>(neighbor)] != 0) {
            continue;
        }

        int block = 0;
        for (const auto& coordinate : vision) {
            if (0 <= std::get<0>(coordinate) + std::get<0>(dir) && std::get<0>(coordinate) + std::get<0>(dir) < rows 
                && 0 <= std::get<1>(coordinate) + std::get<1>(dir) && std::get<1>(coordinate) + std::get<1>(dir) < cols 
                && maze[std::get<0>(coordinate) + std::get<0>(dir)][std::get<1>(coordinate) + std::get<1>(dir)] == -1) {
                block += 1;
            }
        }

        if (block > maxBlock) {
            maxBlock = block;
            move_dir = dir;
        }
    }

    return move_dir;
}

std::tuple<int, int> Direction::final_check(std::tuple<int, int> start) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (maze[i][j] == 2) {
                    auto direct = destination(start, std::make_tuple(i, j));
                    if (direct != std::make_tuple(0, 0)) {
                        return direct;
                    }
                }
            }
        }

        return std::make_tuple(0, 0);  // Goal not found
}

// Return type redef
int* Direction::redef(const std::tuple<int, int>& dir) {
    int* ret = (int*)malloc(2 * sizeof(int));
    if (ret) {
        ret[0] = std::get<0>(dir);
        ret[1] = std::get<1>(dir);
        // Debug
        // std::cout << "ret: " << ret[0] << ", " << ret[1] << std::endl;
    }
    else {
        std::cerr << "Error: Null pointer dereferenced." << std::endl;
    }

    return ret;
}

// DEBUG
// 
// Function to print the contents of the priority queue
void Direction::print_priority_queue(std::priority_queue<
    std::tuple<float, std::tuple<int, int>>,
    std::vector<std::tuple<float, std::tuple<int, int>>>,
    std::greater<>> open_set) {
    // Create a vector to hold the elements of the priority queue
    std::vector<std::tuple<float, std::tuple<int, int>>> elements;

    // Copy the elements of the priority queue to the vector
    while (!open_set.empty()) {
        elements.push_back(open_set.top());
        open_set.pop();
    }

    // Print the elements
    for (const auto& element : elements) {
        float priority = std::get<0>(element);
        auto coords = std::get<1>(element);
        int x = std::get<0>(coords);
        int y = std::get<1>(coords);
        std::cout << "Priority: " << priority << ", Coordinates: (" << x << ", " << y << ")" << std::endl;
    }
}

// Print maze
void Direction::draw_maze() {
    int i, j;
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            std::cout << std::setw(2) << static_cast<int>(maze[i][j]) << " ";
        }
        std::cout << std::endl;
    }
}