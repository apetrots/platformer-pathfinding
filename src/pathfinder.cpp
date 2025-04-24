#include "pathfinder.h"

#include <unordered_map>
#include <godot_cpp/classes/tile_data.hpp>

using namespace godot;

void godot::Pathfinder::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_polygon_2d", "polygon_2d"), &Pathfinder::set_polygon_2d);
    ClassDB::bind_method(D_METHOD("get_polygon_2d"), &Pathfinder::get_polygon_2d);

    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "polygon_2d", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Polygon2D"), "set_polygon_2d", "get_polygon_2d");

	ClassDB::bind_method(D_METHOD("set_nav_region", "nav_region"), &Pathfinder::set_nav_region);
	ClassDB::bind_method(D_METHOD("get_nav_region"), &Pathfinder::get_nav_region);

    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "nav_region", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "NavigationRegion2D"), "set_nav_region", "get_nav_region");
}

void godot::Pathfinder::set_polygon_2d(const NodePath &p_polygon_2d)
{
    if (polygon_2d == p_polygon_2d) {
        return;
    }

    polygon_2d = p_polygon_2d;
}

NodePath godot::Pathfinder::get_polygon_2d() const
{
    return polygon_2d;
}

void Pathfinder::set_nav_region(const NodePath &p_nav_region)
{
    if (nav_region == p_nav_region) {
        return;
    }

    nav_region = p_nav_region;
}

NodePath Pathfinder::get_nav_region() const
{
    return nav_region;
}

godot::Pathfinder::Pathfinder()
{
}

godot::Pathfinder::~Pathfinder()
{
}

void godot::Pathfinder::_ready()
{
    if (!Engine::get_singleton()->is_editor_hint()) {
        // Your game play only code.
        generate_graph();
    }
}

void godot::Pathfinder::_process(double delta)
{
}

void godot::Pathfinder::find_path(Vector2 from, Vector2 to)
{
}

struct Edge
{
    int32_t p1_idx;
    int32_t p2_idx;
};

void godot::Pathfinder::generate_graph()
{
    Polygon2D *polygon_2d_node = Object::cast_to<Polygon2D>(get_node_or_null(polygon_2d));
    ERR_FAIL_NULL_MSG(polygon_2d_node, "Polygon2D is invalid: '" + polygon_2d + "'");

    NavigationRegion2D *nav_region_node = Object::cast_to<NavigationRegion2D>(get_node_or_null(nav_region));
    ERR_FAIL_NULL_MSG(nav_region_node, "Navigation region is invalid: '" + nav_region + "'");   

    Ref<NavigationPolygon> nav_polygon = nav_region_node->get_navigation_polygon();
    ERR_FAIL_NULL_MSG(nav_polygon, "Navigation polygon is invalid: '" + nav_region + "'"); 
    
    int32_t poly_count = nav_polygon->get_polygon_count();
    // maps smaller vertex index of an edge to a map of the other edge's vertice to the # of polygons that use that edge
    std::unordered_map<int32_t, std::unordered_map<int32_t, int8_t>> half_edges;
    for (int32_t i = 0; i < poly_count; i++)
    {
        PackedInt32Array idxs = nav_polygon->get_polygon(i);
        for (int32_t j = 0; j < idxs.size(); j++)
        {
            int32_t current = idxs[j];
            int32_t next = idxs[(j + 1) % idxs.size()];
            int32_t min_idx = std::min(current, next);
            int32_t max_idx = std::max(current, next);

            half_edges[min_idx][max_idx]++;
        }
    }

    PackedVector2Array outline = nav_polygon->get_outline(0);

    PackedVector2Array vertices = nav_polygon->get_vertices();
    std::vector<Edge> edges;
    for (int32_t i = 0; i < half_edges.size(); i++)
    {
        for (int32_t j = 0; j < half_edges[i].size(); j++)
        {
            if (half_edges[i][j] == 1) // only one polygon uses this edge
            {
                bool outline_edge = false;
                for (int o_idx = 0; o_idx < outline.size(); o_idx++)
                {
                    if (outline[o_idx].distance_squared_to(vertices[i]) < 0.001 
                        || outline[o_idx].distance_squared_to(vertices[j]) < 0.001)
                    {
                        outline_edge = true;
                    }
                }

                if (outline_edge)
                    continue;

                Edge new_edge = {
                    i, j,
                };
                edges.push_back(new_edge);
            }
        }
    }

    std::vector<PackedInt32Array> islands;
    
    std::unordered_map<int32_t, std::vector<int32_t>> adjacency_list;
    for (const Edge &edge : edges)
    {
        adjacency_list[edge.p1_idx].push_back(edge.p2_idx);
        adjacency_list[edge.p2_idx].push_back(edge.p1_idx);
    }

    std::unordered_map<int32_t, bool> visited;
    for (const auto &pair : adjacency_list)
    {
        visited[pair.first] = false;
    }

    for (const auto &pair : adjacency_list)
    {
        if (!visited[pair.first])
        {
            PackedInt32Array island;
            std::vector<int32_t> stack = {pair.first};

            while (!stack.empty())
            {
                int32_t current = stack.back();
                stack.pop_back();

                if (visited[current])
                {
                    continue;
                }

                visited[current] = true;
                island.push_back(current);

                for (int32_t neighbor : adjacency_list[current])
                {
                    if (!visited[neighbor])
                    {
                        stack.push_back(neighbor);
                    }
                }
            }

            islands.push_back(island);
        }
    }

    Array islands_array;
    for (auto &island : islands)
    {
        islands_array.append(island);
    }
    
    polygon_2d_node->set_polygon(vertices);
    polygon_2d_node->set_polygons(islands_array);

    // {
        // Ref<AStar2D> graph = memnew(AStar2D);
        // graph->set_name("PathfinderGraph");

        // for (int32_t i = 0; i < island.size(); i++)
        // {
        //     int32_t vertex_idx = island[i];
        //     Vector2 vertex = vertices[vertex_idx];
        //     graph->add_node(vertex_idx, vertex);
        // }

        // for (int32_t i = 0; i < island.size(); i++)
        // {
        //     int32_t vertex_idx = island[i];
        //     Vector2 vertex = vertices[vertex_idx];

        //     for (int32_t j = 0; j < adjacency_list[vertex_idx].size(); j++)
        //     {
        //         int32_t neighbor_idx = adjacency_list[vertex_idx][j];
        //         Vector2 neighbor_vertex = vertices[neighbor_idx];

        //         float distance = vertex.distance_to(neighbor_vertex);
        //         graph->connect_nodes(vertex_idx, neighbor_idx, distance);
        //     }
        // }

        // graph->set_name("PathfinderGraph" + String::num_int64(island.size()));
    // }
}
