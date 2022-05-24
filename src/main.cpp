#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    //Complete this TODO to satisfy Project Rubric Criterias of User Input
  
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    float start_x = 0;
    float start_y = 0;
    float end_x = 0;
    float end_y = 0;

    std::cout << "Enter start x start y end x end y as 4 separate numbers between 0 and 100:\n";
    // std::cin >> start_x;
    std::cin >> start_x >> start_y >> end_x >> end_y;


std::cout << "DEBUG: start_x " << start_x << "\n";
std::cout << "DEBUG: start_y " << start_y << "\n";
std::cout << "DEBUG: end_x " << end_x << "\n";
std::cout << "DEBUG: end_y " << end_y << "\n";

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    //
    // testing
    //
    // RouteModel::Node* test_node = &model.FindClosestNode(start_x, start_y);
    // std::cout << "Test node: " << test_node << "\n";
    // CalculateHValue test
    // std::cout << "TESTING: CalculateHValue: " << route_planner.CalculateHValue(test_node) << "\n";
    // route_planner.AddNeighbors(test_node);
    // auto neighbors = test_node->neighbors;
    // Check results for each neighbor.
    // std::cout << "TESTING: neighbors->size " << neighbors.size() << "\n";
    // for (int i = 0; i < neighbors.size(); i++) {
    //     std::cout << "TESTING: neighbors->parent " << neighbors[i]->parent << "\n";
    //     std::cout << "TESTING: neighbors->g " << neighbors[i]->g_value << "\n";
    //     std::cout << "TESTING: neighbors->h " << neighbors[i]->h_value << "\n";
    //     std::cout << "TESTING: neighbors->visited " << neighbors[i]->visited << "\n";
    // }
    // Check NextNode
    // RouteModel::Node* test_next_node = route_planner.NextNode();
    // std::cout << "TESTING: NextNOde parent " << test_next_node->parent << "\n";
    // std::cout << "TESTING: NextNOde g " << test_next_node->g_value << "\n";
    // std::cout << "TESTING: NextNOde h " << test_next_node->h_value << "\n";
    // std::cout << "TESTING: NextNOde visited " << test_next_node->visited << "\n";
    // Check ConstructPath - Not working
    // RouteModel::Node* test_end_node = route_planner.NextNode();
    // std::cout << "Test end node: " << test_end_node << "\n";
    // test_end_node->parent = test_node;
    // std::vector<RouteModel::Node> test_path = route_planner.ConstructFinalPath(test_end_node);
    // Test the path.
    // std::cout << "TESTING: ConstructFinalPath size " << test_path.size() << "\n";
    // std::cout << "TESTING: ConstructFinalPath front x " << test_path.front().x << "\n";
    // std::cout << "TESTING: ConstructFinalPath front y " << test_path.front().y << "\n";
    // std::cout << "TESTING: ConstructFinalPath back x " << test_path.back().x << "\n";
    // std::cout << "TESTING: ConstructFinalPath back y " << test_path.back().y << "\n";
    //
    // testing
    //
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
