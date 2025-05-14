#include "pathfinder.h"

#include <unordered_map>
#include <godot_cpp/classes/mesh_instance2d.hpp>
#include <godot_cpp/classes/immediate_mesh.hpp>
#include <godot_cpp/classes/tile_data.hpp>
#include <godot_cpp/classes/geometry2d.hpp>

using namespace godot;

void godot::Pathfinder::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_debug_draw", "debug_draw"), &Pathfinder::set_debug_draw);
    ClassDB::bind_method(D_METHOD("get_debug_draw"), &Pathfinder::get_debug_draw);

    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "debug_draw", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MeshInstance2D"), "set_debug_draw", "get_debug_draw");

	ClassDB::bind_method(D_METHOD("set_nav_region", "nav_region"), &Pathfinder::set_nav_region);
	ClassDB::bind_method(D_METHOD("get_nav_region"), &Pathfinder::get_nav_region);

    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "nav_region", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "NavigationRegion2D"), "set_nav_region", "get_nav_region");
}

void godot::Pathfinder::set_debug_draw(const NodePath &p_debug_draw)
{
    if (debug_draw == p_debug_draw) {
        return;
    }

    debug_draw = p_debug_draw;
}

NodePath godot::Pathfinder::get_debug_draw() const
{
    return debug_draw;
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
    MeshInstance2D *debug_draw_node = Object::cast_to<MeshInstance2D>(get_node_or_null(debug_draw));
    ERR_FAIL_NULL_MSG(debug_draw_node, "MeshInstance2D is invalid: '" + debug_draw + "'");

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

    std::vector<PackedInt32Array> island_idxs;
    
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

            island_idxs.push_back(island);
        }
    }

    Ref<ImmediateMesh> mesh = debug_draw_node->get_mesh();
    mesh->surface_begin(Mesh::PRIMITIVE_LINES);

    // TODO: Make this a setting. Might just want to update sometimes.
    islands.clear();

    islands.reserve(island_idxs.size());
    for (auto &idxs : island_idxs)
    {
        PackedVector2Array pts;
        pts.resize(idxs.size());
        
        for (uint32_t i = 0; i < idxs.size(); i++)
        {
            pts[i] = vertices[idxs[i]];
        }

        islands.push_back(pts);
    }

    // Generate the walkable surface points, put them into AStar2D graph and island_surface_pts
    graph.instantiate();
    int64_t node_id = 0;
    for (auto &island : islands)
    {
        PackedVector2Array surf_pts;
        bool connect_next_pt = false;
        for (uint32_t i = 1; i < island.size(); i++)
        {
            uint32_t j = i-1;
            Vector2 pt1 = island[j];
            Vector2 pt2 = island[i];

            // Calculate the outward normal
            Vector2 normal = (pt2 - pt1).normalized().orthogonal();
            Vector2 mid_point = (pt1 + pt2) / 2.0f;
            
            // TODO: Might want to double check the 0.01f offset multiplier on the normal.
            if (Geometry2D::get_singleton()->is_point_in_polygon(mid_point + normal * 0.01f, island))
            {
                normal = -normal;
            }

            float angle_deg = ABS(normal.angle_to(Vector2(0, -1.0f)) * 180.0f / Math_PI);
            print_line("Angle: " + String::num(angle_deg));
            // make sure the angle is not past the max walkable surface angle
            if (angle_deg > max_walkable_surface_angle)
                continue;

            // add points along the edge to where they are surface_subdivision_distance apart
            Vector2 edge = pt2 - pt1;
            uint32_t subdivisions = ceil(edge.length() / max_surface_subdivision_distance);
            for (uint32_t i = 0; i < subdivisions; i++)
            {
                Vector2 new_pt = pt1 + edge * ((real_t)i / (real_t)subdivisions);
                surf_pts.push_back(new_pt);
                graph->add_point(node_id, new_pt, 1.0f);
                
                print_line("Adding point: " + String::num(new_pt.x) + ", " + String::num(new_pt.y));

                if (connect_next_pt)
                    graph->connect_points(node_id, node_id - 1);
                connect_next_pt = true;

                node_id++;
            }

            // TODO: only if wanting to debug draw...
            if (true)
            {
                mesh->surface_set_color(Color(1, 0, 0));
                mesh->surface_add_vertex_2d(pt1);
                mesh->surface_set_color(Color(1, 0, 0));
                mesh->surface_add_vertex_2d(pt2);
            }
        }

        island_surface_pts.push_back(surf_pts);
    }

    mesh->surface_end();
}
