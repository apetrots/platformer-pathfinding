#include "pathfinder.h"

#include <unordered_map>
#include <godot_cpp/classes/mesh_instance2d.hpp>
#include <godot_cpp/classes/immediate_mesh.hpp>
#include <godot_cpp/classes/tile_data.hpp>
#include <godot_cpp/classes/geometry2d.hpp>
#include <algorithm>

using namespace godot;

void godot::Pathfinder::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_debug_draw", "debug_draw"), &Pathfinder::set_debug_draw);
    ClassDB::bind_method(D_METHOD("get_debug_draw"), &Pathfinder::get_debug_draw);

    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "debug_draw", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "MeshInstance2D"), "set_debug_draw", "get_debug_draw");

	ClassDB::bind_method(D_METHOD("set_nav_region", "nav_region"), &Pathfinder::set_nav_region);
	ClassDB::bind_method(D_METHOD("get_nav_region"), &Pathfinder::get_nav_region);

    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "nav_region", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "NavigationRegion2D"), "set_nav_region", "get_nav_region");

    ClassDB::bind_method(D_METHOD("find_path", "from", "to"), &Pathfinder::find_path);
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

void godot::Pathfinder::_process(double delta)
{
    if (!Engine::get_singleton()->is_editor_hint() && to_generate_graph) {
        generate_graph();
        to_generate_graph = false;
    }
}

godot::TypedArray<PathAction> godot::Pathfinder::find_path(Vector2 from, Vector2 to)
{
    if (graph.is_null())
    {
        print_line("Graph is null!");
        return {};
    }

    int64_t from_id = graph->get_closest_point(from);
    int64_t to_id = graph->get_closest_point(to);

    if (from_id == -1 || to_id == -1)
    {
        print_line("Invalid point!");
        return {};
    }

    PackedInt64Array path_ids = graph->get_id_path(from_id, to_id);

    if (path_ids.size() == 0)
    {
        return {};
    }

    MeshInstance2D *debug_draw_node = Object::cast_to<MeshInstance2D>(get_node_or_null(debug_draw));
    // ERR_FAIL_NULL_MSG(debug_draw_node, "MeshInstance2D is invalid: '" + debug_draw + "'");

    Ref<ImmediateMesh> mesh = debug_draw_node->get_mesh();
    mesh->clear_surfaces();
    mesh->surface_begin(Mesh::PRIMITIVE_LINE_STRIP);

    TypedArray<PathAction> path_actions;

    Vector2 start_pt = graph->get_point_position(path_ids[0]);
    mesh->surface_add_vertex_2d(start_pt);

    Vector2 last_pt = from;
    for (int64_t node_id : path_ids)
    {
        Vector2 pt = graph->get_point_position(node_id);
        auto scale = graph->get_point_weight_scale(node_id);

        Ref<PathAction> action = {};
        action.instantiate();

        auto jump_info = graph->get_jump_info(node_id);
        if (jump_info != NULL)
        {
            Vector2 delta_pos = jump_info->to - jump_info->from;
            Vector2 start_pos(jump_info->from.x, jump_info->from.y - agent_size.height / 2.0f);
            Vector2 end_pos = start_pos + delta_pos;
            Vector2 jump_velocity = delta_pos/jump_info->duration - acceleration * jump_info->duration * 0.5f;

            action->is_jump = true;
            action->from = start_pos;
            action->jump_velocity = jump_velocity;
            action->to = end_pos;
            last_pt = end_pos;

            mesh->surface_set_color(Color(0, 1, 0));
            mesh->surface_add_vertex_2d(jump_info->from);
            mesh->surface_set_color(Color(0, 0, 1));
            mesh->surface_add_vertex_2d(start_pos);
            for (real_t t = 0.0f; t < jump_info->duration; t += 1.0/(real_t)5.0)
            {
                Vector2 pos = start_pos + jump_velocity * t + 0.5f * acceleration * t * t;
                mesh->surface_add_vertex_2d(pos);
            }
            mesh->surface_add_vertex_2d(end_pos);
            mesh->surface_set_color(Color(0, 1, 0));
            mesh->surface_add_vertex_2d(jump_info->to);
        }
        else
        {
            Vector2 pt = graph->get_point_position(node_id);
            mesh->surface_set_color(Color(0, 1, 0));
            mesh->surface_add_vertex_2d(pt);
            
            action->is_jump = false;
            action->from = last_pt;
            action->to = pt;
            last_pt = pt;
        }

        path_actions.push_back(action);
    }

    Ref<PathAction> last = path_actions[path_actions.size() - 1];
    if (!last->is_jump)
        last->to = to;

    mesh->surface_end();

    return path_actions;
}

struct Edge
{
    int32_t p1_idx;
    int32_t p2_idx;
};

struct JumpNodeInfo
{
    real_t magnitude_squared; 
    real_t duration = -1.0f;
    Vector2 to, from;
    uint32_t surf1_edge_idx, surf2_edge_idx;
};

void godot::Pathfinder::generate_graph()
{
    MeshInstance2D *debug_draw_node = Object::cast_to<MeshInstance2D>(get_node_or_null(debug_draw));
    ERR_FAIL_NULL_MSG(debug_draw_node, "MeshInstance2D is invalid: '" + debug_draw + "'");

    NavigationRegion2D *nav_region_node = Object::cast_to<NavigationRegion2D>(get_node_or_null(nav_region));
    ERR_FAIL_NULL_MSG(nav_region_node, "Navigation region is invalid: '" + nav_region + "'");
    nav_region_node->bake_navigation_polygon(false);

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
    for (auto &island : islands)
    {
        // for special case where the starting edge is walkable, set this to the node_id there, 
        // so we can loop back and connect the end edge to it if its also walkable. 
        int64_t start_node_id = -1;

        PackedVector2Array surf_pts;
        PackedInt64Array surf_node_ids;
        int64_t last_node = -1;
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
            // make sure the angle is not past the max walkable surface angle
            if (angle_deg > max_walkable_surface_angle)
            {
                // end of continuous walkable surface
                last_node = -1;

                if (surf_pts.size() > 0)
                {
                    print_line("new surface! " + String::num_int64(surf_pts.size()));
                    island_surface_pts.push_back(surf_pts);
                    island_surface_node_ids.push_back(surf_node_ids);
                    surf_pts.clear();
                    surf_node_ids.clear();
                }
                
                continue;
            }
            
       
            if (last_node != -1)
            {
                auto node_id = graph->get_point_count();
                graph->add_point(node_id, pt2);
                graph->connect_points(last_node, node_id);

                // TODO testing
                surf_pts.push_back(pt2);
                surf_node_ids.push_back(node_id);
                
                last_node = node_id;
            }
            else
            {
                auto node_id = graph->get_point_count();
                if (i == 1)
                {
                    start_node_id = node_id;
                    print_line("Start node: " + String::num(start_node_id));
                }
    
                graph->add_point(node_id, pt1);
                graph->add_point(node_id+1, pt2);

                graph->connect_points(node_id, node_id + 1);

                // // TODO testing
                surf_pts.push_back(pt1);
                surf_node_ids.push_back(node_id);
                surf_pts.push_back(pt2);
                surf_node_ids.push_back(node_id + 1);

                // start or keep the continuous walkable surface going...
                last_node = node_id + 1;

            }
            
            // last edge being tested, walkable and the first edge was also walkable...
            if (i == island.size() - 1 && start_node_id != -1)
                graph->connect_points(last_node, start_node_id);

            // TODO: only if wanting to debug draw...
            if (true)
            {
                mesh->surface_set_color(Color(1, 0, 0));
                mesh->surface_add_vertex_2d(pt1);
                mesh->surface_set_color(Color(1, 0, 0));
                mesh->surface_add_vertex_2d(pt2);
            }
        }
        if (surf_pts.size() > 0)
        {
            print_line("new surface (at the end)! " + String::num_int64(surf_pts.size()));
            island_surface_pts.push_back(surf_pts);
            island_surface_node_ids.push_back(surf_node_ids);
            surf_pts.clear();
            surf_node_ids.clear();
        }
    }

    // move edges inward until we sized agent can actually stand with it's center there.
    {
        Ref<World2D> world = get_viewport()->get_world_2d();
        PhysicsDirectSpaceState2D *space_state = world->get_direct_space_state();
    
        Ref<RectangleShape2D> rect_shape;
        rect_shape.instantiate();
        rect_shape->set_size(agent_size);
    
        Ref<PhysicsShapeQueryParameters2D> params;
        params.instantiate();
        params->set_shape(rect_shape);

        for (int32_t surf_idx = 0; surf_idx < island_surface_pts.size(); surf_idx++)
        {
            auto &surf = island_surface_pts[surf_idx];
            auto &node_ids = island_surface_node_ids[surf_idx];
            
            // only contract the edges on either end of a surface, we know the middle is all walkable.
            {
                Vector2 surf_pt1 = surf[0];
                Vector2 surf_pt2 = surf[1];
                Vector2 surf_edge = surf_pt2 - surf_pt1;

                Vector2 pos = surf_pt1 - Vector2(0, agent_size.height / 2.0f);
                print_line("pos: " + String::num(pos.x) + ", " + String::num(pos.y));

                params->set_transform(Transform2D(0.0f, pos));
                
                // away from collision is surf_edge.normalized()
                if (space_state->intersect_shape(params, 1).size() > 0)
                {
                    print_line("move inward 1!");
                    // If collision, move further inward.
                    surf[0] = surf_pt1 + surf_edge.normalized() * (agent_size.width / 2.0f);
                    int64_t node_id = node_ids[0];
                    graph->set_point_position(node_id, surf[0]);
                }
            }
            {
                Vector2 surf_pt1 = surf[surf.size() - 1];
                Vector2 surf_pt2 = surf[surf.size() - 2];
                Vector2 surf_edge = surf_pt2 - surf_pt1;

                Vector2 pos = surf_pt1 - Vector2(0, agent_size.height / 2.0f);

                params->set_transform(Transform2D(0.0f, pos));
                
                // away from collision is surf_edge.normalized()
                if (space_state->collide_shape(params, 1).size() > 0)
                {
                    print_line("move inward 2!");
                    // If collision, move further inward.
                    surf[surf.size() - 1] = surf_pt1 + surf_edge.normalized() * (agent_size.width / 2.0f);
                    int64_t node_id = node_ids[surf.size() - 1];
                    graph->set_point_position(node_id, surf[surf.size() - 1]);
                }
            }
        }
    }

    // print_line("surfaces: " + String::num_int64(island_surface_pts.size()) + "\n");

    // Add jump connections between islands.
    for (int32_t surf1_idx = 0; surf1_idx < island_surface_pts.size(); surf1_idx++)
    {
        for (int32_t surf2_idx = surf1_idx+1; surf2_idx < island_surface_pts.size(); surf2_idx++)
        {
            auto &surf1 = island_surface_pts[surf1_idx];
            auto &surf2 = island_surface_pts[surf2_idx];
            
            // compare each edge of surf1 to each edge of surf2
            // TODO do this for each agent type, so we can have different hitboxes for different agents?

            // TODO: Make this a class, so we can save info for later creating the jump connections with knowledge
            // of the walkable nodes it has to connect to on either side.
            // TODO Need these properties for each direction (surf1 -> surf2 and surf2 -> surf1)

            // direction 1 (surf1 -> surf2)
            JumpNodeInfo lowest_jump1;
            lowest_jump1.magnitude_squared = INFINITY;
            lowest_jump1.duration = -1.0f;

            // direction 2 (surf2 -> surf1)
            JumpNodeInfo lowest_jump2;
            lowest_jump2.magnitude_squared = INFINITY;
            lowest_jump2.duration = -1.0f;
            for (uint32_t i = 1; i < surf1.size(); i++)
            {
                Vector2 surf1_pt1 = surf1[i-1];
                Vector2 surf1_pt2 = surf1[i];
                Vector2 surf1_edge = surf1_pt2 - surf1_pt1;
                
                for (uint32_t j = 1; j < surf2.size(); j++)
                {
                    Vector2 surf2_pt1 = surf2[j-1];
                    Vector2 surf2_pt2 = surf2[j];

                    PackedVector2Array closest_points = Geometry2D::get_singleton()->get_closest_points_between_segments(surf1_pt1, surf1_pt2, surf2_pt1, surf2_pt2);
                    Vector2 closest_pt1 = closest_points[0];
                    Vector2 closest_pt2 = closest_points[1];

                    // quick filter
                    if (closest_pt1.distance_squared_to(closest_pt2) > max_jump_distance * max_jump_distance)
                        continue;

                    Vector2 surf2_edge = surf2_pt2 - surf2_pt1;
                    
                    // TODO maybe i dont need to subdivide the edges if we can find another way to get closest points on the edges between eachotherg
                    // test from the point closest to the point we're jumping from out (or that point but offset by the hitbox distance)
                    
                    // Geometry2D::get_singleton()->get_closest_points_between_segments(surf1_pt1, surf1_pt2, surf2_pt1, surf2_pt2)

                    // DIRECTION 1 (surf1 -> surf2)
                
                    // add points along the edge to where they are surface_subdivision_distance apart
                    uint32_t subdivisions = ceilf((real_t)surf1_edge.length() / (real_t)max_surface_subdivision_distance);
                    for (uint32_t subd = 0; subd < subdivisions; subd++)
                    {
                        // TODO start subdivision search from the middle of the edge
                        // print_line("subd: " + String::num(subd) + " of " + String::num(subdivisions));
                        Vector2 jump_from = surf1_pt1 + surf1_edge * ((real_t)subd / (real_t)subdivisions);
                        Vector2 jump_to = Geometry2D::get_singleton()->get_closest_point_to_segment(jump_from, surf2_pt1, surf2_pt2);
                        
                        Vector2 out_velocity;
                        float jump_duration = try_find_unobstructed_jump(out_velocity, jump_from, jump_to, 5);
                        if (jump_duration > 0.0f)
                        {
                            if (out_velocity.length_squared() < lowest_jump1.magnitude_squared)
                            {
                                lowest_jump1.magnitude_squared = out_velocity.length_squared();
                                lowest_jump1.duration = jump_duration;
                                lowest_jump1.from = jump_from;
                                lowest_jump1.to = jump_to;
                                lowest_jump1.surf1_edge_idx = i;
                                lowest_jump1.surf2_edge_idx = j;
                            }
                        }
                    }

                    // DIRECTION 2 (surf2 -> surf1)
                
                    // add points along the edge to where they are surface_subdivision_distance apart
                    subdivisions = ceilf((real_t)surf2_edge.length() / (real_t)max_surface_subdivision_distance);
                    for (uint32_t subd = 0; subd < subdivisions; subd++)
                    {
                        // TODO start subdivision search from the middle of the edge
                        // print_line("subd: " + String::num(subd) + " of " + String::num(subdivisions));
                        Vector2 jump_from = surf2_pt1 + surf2_edge * ((real_t)subd / (real_t)subdivisions);
                        Vector2 jump_to = Geometry2D::get_singleton()->get_closest_point_to_segment(jump_from, surf1_pt1, surf1_pt2);
                        
                        Vector2 out_velocity;
                        float jump_duration = try_find_unobstructed_jump(out_velocity, jump_from, jump_to, 5);
                        if (jump_duration > 0.0f)
                        {
                            if (out_velocity.length_squared() < lowest_jump2.magnitude_squared)
                            {
                                lowest_jump2.magnitude_squared = out_velocity.length_squared();
                                lowest_jump2.duration = jump_duration;
                                lowest_jump2.from = jump_from;
                                lowest_jump2.to = jump_to;
                                lowest_jump2.surf1_edge_idx = i;
                                lowest_jump2.surf2_edge_idx = j;
                            }
                        }
                    }
                }
            }

            if (lowest_jump1.duration > 0.0f)
            {
                // print_line("Jump from: " + String::num(lowest_jump1.from.x) + ", " + String::num(lowest_jump1.from.y) + " to: " + String::num(lowest_jump1.to.x) + ", " + String::num(lowest_jump1.to.y));
                // print_line("jump duration: " + String::num(lowest_jump1.duration));
                // print_line("with jump velocity: " + String::num(lowest_jump_velocity.x) + ", " + String::num(lowest_jump_velocity.y));
                // print_line("edge1: " + String::num(surf1_edge_idx));
                // print_line("edge2: " + String::num(surf2_edge_idx));
                // print_line("surf1: " + String::num(surf1_idx) + " of " + String::num(island_surface_node_ids.size()));
                // print_line("surf2: " + String::num(surf2_idx) + " of " + String::num(island_surface_node_ids.size()));
                // print_line("surfaces: " + String::num_int64(island_surface_pts.size()) + "\n");

                int64_t edge1_node1 = island_surface_node_ids[surf1_idx][lowest_jump1.surf1_edge_idx];
                int64_t edge1_node2 = island_surface_node_ids[surf1_idx][lowest_jump1.surf1_edge_idx - 1];
   
                int64_t edge2_node1 = island_surface_node_ids[surf2_idx][lowest_jump1.surf2_edge_idx];
                int64_t edge2_node2 = island_surface_node_ids[surf2_idx][lowest_jump1.surf2_edge_idx - 1];


                int64_t jump_from_node = graph->get_point_count(); 
                graph->add_point(jump_from_node, lowest_jump1.from);
                graph->connect_points(jump_from_node, edge1_node1);
                graph->connect_points(jump_from_node, edge1_node2);
                // TODO is disconnecting necessary?
                graph->disconnect_points(edge1_node1, edge1_node2);
                
                int64_t jump_to_node = graph->get_point_count(); 
                graph->add_point(jump_to_node, lowest_jump1.to);
                graph->connect_points(jump_to_node, edge2_node1);
                graph->connect_points(jump_to_node, edge2_node2);
                // TODO is disconnecting necessary?
                graph->disconnect_points(edge2_node1, edge2_node2);
                
                graph->add_jump_node(jump_from_node, jump_to_node, lowest_jump1.duration);

                // debug draw jump
                Vector2 delta_pos = lowest_jump1.to - lowest_jump1.from;
                Vector2 start_pos(lowest_jump1.from.x, lowest_jump1.from.y - agent_size.height / 2.0f);
                Vector2 end_pos = start_pos + delta_pos;
                Vector2 jump_velocity = delta_pos/lowest_jump1.duration - acceleration * lowest_jump1.duration * 0.5f;
                print_line("Jump velocity: " + String::num(jump_velocity.x) + ", " + String::num(jump_velocity.y));

                mesh->surface_set_color(Color(0, 1, 0));
                mesh->surface_add_vertex_2d(start_pos);
                mesh->surface_add_vertex_2d(start_pos + Vector2(0, -1.0));
                mesh->surface_set_color(Color(0, 0, 1));
                mesh->surface_add_vertex_2d(start_pos);
                for (real_t t = 0.0f; t < lowest_jump1.duration; t += 1.0/(real_t)5.0)
                {
                    Vector2 pos = start_pos + jump_velocity * t + 0.5f * acceleration * t * t;
                    mesh->surface_add_vertex_2d(pos);
                    mesh->surface_add_vertex_2d(pos);
                }
                mesh->surface_add_vertex_2d(end_pos);
                
                mesh->surface_set_color(Color(0, 1, 1));
                mesh->surface_add_vertex_2d(start_pos);
                mesh->surface_set_color(Color(1, 1, 0));
                mesh->surface_add_vertex_2d(end_pos);
            }


            if (lowest_jump2.duration > 0.0f)
            {
                print_line("Jump from: " + String::num(lowest_jump2.from.x) + ", " + String::num(lowest_jump2.from.y) + " to: " + String::num(lowest_jump2.to.x) + ", " + String::num(lowest_jump2.to.y));
                print_line("jump duration: " + String::num(lowest_jump2.duration));

                int64_t edge1_node1 = island_surface_node_ids[surf1_idx][lowest_jump2.surf1_edge_idx];
                int64_t edge1_node2 = island_surface_node_ids[surf1_idx][lowest_jump2.surf1_edge_idx - 1];
   
                int64_t edge2_node1 = island_surface_node_ids[surf2_idx][lowest_jump2.surf2_edge_idx];
                int64_t edge2_node2 = island_surface_node_ids[surf2_idx][lowest_jump2.surf2_edge_idx - 1];

                int64_t jump_from_node = graph->get_point_count(); 
                graph->add_point(jump_from_node, lowest_jump2.from);
                graph->connect_points(jump_from_node, edge2_node1);
                graph->connect_points(jump_from_node, edge2_node2);
                // TODO is disconnecting necessary?
                graph->disconnect_points(edge2_node1, edge2_node2);
                
                int64_t jump_to_node = graph->get_point_count(); 
                graph->add_point(jump_to_node, lowest_jump2.to);
                graph->connect_points(jump_to_node, edge1_node1);
                graph->connect_points(jump_to_node, edge1_node2);
                // TODO is disconnecting necessary?
                graph->disconnect_points(edge1_node1, edge1_node2);
                
                graph->add_jump_node(jump_from_node, jump_to_node, lowest_jump2.duration);

                // debug draw jump
                Vector2 delta_pos = lowest_jump2.to - lowest_jump2.from;
                Vector2 start_pos(lowest_jump2.from.x, lowest_jump2.from.y - agent_size.height / 2.0f);
                Vector2 end_pos = start_pos + delta_pos;
                Vector2 jump_velocity = delta_pos/lowest_jump2.duration - acceleration * lowest_jump2.duration * 0.5f;
                print_line("Jump velocity: " + String::num(jump_velocity.x) + ", " + String::num(jump_velocity.y));

                mesh->surface_set_color(Color(0, 1, 0));
                mesh->surface_add_vertex_2d(start_pos);
                mesh->surface_add_vertex_2d(start_pos + Vector2(0, -1.0));
                mesh->surface_set_color(Color(0, 0, 1));
                mesh->surface_add_vertex_2d(start_pos);
                for (real_t t = 0.0f; t < lowest_jump2.duration; t += 1.0/(real_t)5.0)
                {
                    Vector2 pos = start_pos + jump_velocity * t + 0.5f * acceleration * t * t;
                    mesh->surface_add_vertex_2d(pos);
                    mesh->surface_add_vertex_2d(pos);
                }
                mesh->surface_add_vertex_2d(end_pos);
                
                mesh->surface_set_color(Color(0, 1, 1));
                mesh->surface_add_vertex_2d(start_pos);
                mesh->surface_set_color(Color(1, 1, 0));
                mesh->surface_add_vertex_2d(end_pos);
            }
        }
    }



    mesh->surface_end();
}

float godot::Pathfinder::try_find_unobstructed_jump(Vector2& out_velocity, Vector2 from, Vector2 to, int test_intervals)
{
    out_velocity = Vector2(0.0, 0.0);
    float jump_duration = -1.0f;

    Ref<World2D> world = get_viewport()->get_world_2d();
    PhysicsDirectSpaceState2D *space_state = world->get_direct_space_state();

    Ref<RectangleShape2D> rect_shape;
    rect_shape.instantiate();
    rect_shape->set_size(agent_size);

    Ref<PhysicsShapeQueryParameters2D> params;
    params.instantiate();
    params->set_shape(rect_shape);
    params->set_collision_mask(1);

    Vector2 delta_pos = to - from;

    // start at lowest energy jump
    jump_duration = sqrtf(sqrtf(4.0 * delta_pos.dot(delta_pos) / acceleration.dot(acceleration)));
    // print_line("trying to find jump velocity with duration: " + String::num(jump_duration) + " with delta pos: " + String::num(delta_pos.x) + ", " + String::num(delta_pos.y));

    Vector2 jump_velocity = delta_pos/jump_duration - acceleration * jump_duration * 0.5f;

    Vector2 start_pos(from.x, from.y - agent_size.height / 2.0f);

    for (real_t t = 0.0f; t < jump_duration; t += 1.0/(real_t)collision_tests_per_second)
    {
        Vector2 pos = start_pos + jump_velocity * t + 0.5f * acceleration * t * t;
        
        // TODO: variable collision mask
        params->set_collision_mask(1);

        params->set_transform(Transform2D(0.0f, pos));
        if (space_state->collide_shape(params, 1).size() > 0)
        {
            // print_line("Jump blocked at time " + String::num(t) + " out of " + String::num(jump_duration));
            return -1.0f;
        }   
    }

    out_velocity = jump_velocity;
    return jump_duration;
}