// direction.h
//
#ifndef DIRECTION_H
#define DIRECTION_H

#include <queue>
#include <cmath>
#include <tuple>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <unordered_map>

class Direction {
public:
    bool toggle;  // Dest rush flag
    int rows, cols;  // Maze size
    std::vector<std::vector<int8_t>> maze;  // Maze info

    // Constructor
    Direction(int row, int column);

    void path_clr();
    void path_mod(const std::vector<std::tuple<int, int>>& path);
    std::tuple<int, int> path_pop(const std::tuple<int, int>& loc);
    float heuristic(const std::tuple<int, int>& a, const std::tuple<int, int>& b);  // Calculate the distance of the two blocks
    std::tuple<int, int> block_search(const std::tuple<int, int>& start, const std::tuple<int, int>& goal);  // Find specified block across the maze
    std::tuple<int, int> destination(const std::tuple<int, int>& start, const std::tuple<int, int>& goal);  // Dest rush
    std::tuple<int, int> nearest_bblock_path(const std::tuple<int, int>& loc);  // Locate the nearest black block
    std::tuple<int, int> direct(std::tuple<int, int> loc);
    std::tuple<int, int> final_check(std::tuple<int, int> start);

    // Vision
    std::tuple<int, int> visualize(const std::vector<int>& x_buffer, const std::vector<int>& y_buffer, const std::vector<int8_t>& v_buffer, int length);
    
    // Return type redef
    int* redef(const std::tuple<int, int>& dir);

    // Debug
    void print_priority_queue(std::priority_queue<
        std::tuple<float, std::tuple<int, int>>,
        std::vector<std::tuple<float, std::tuple<int, int>>>,
        std::greater<>> open_set);

    void draw_maze();

private:
    std::vector<std::tuple<int, int>> dirs = {{-1,-1}, {-1, 0}, {-1, 1}, 
                                              { 0,-1},          { 0, 1}, 
                                              { 1,-1}, { 1, 0}, { 1, 1}};
    std::vector<std::tuple<int, int>> vision;
    std::vector<std::tuple<int, int>> path;
    std::priority_queue<
        std::tuple<int, std::tuple<int, int>>,
        std::vector<std::tuple<int, std::tuple<int, int>>>,
        std::greater<>> path_tmp;

    // Define a custom hash function for std::tuple<int, int>
    struct tuple_hash {
        template <class T1, class T2>
        std::size_t operator()(const std::tuple<T1, T2>& tuple) const {
            auto h1 = std::hash<T1>{}(std::get<0>(tuple));
            auto h2 = std::hash<T2>{}(std::get<1>(tuple));
            return h1 ^ (h2 << 1); // Combine the two hash values
        }
    };
};

#endif