#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <godot_cpp/classes/polygon2d.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/classes/a_star2d.hpp>
#include <godot_cpp/classes/navigation_region2d.hpp>
#include <godot_cpp/classes/navigation_polygon.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world2d.hpp>
#include <godot_cpp/classes/physics_direct_space_state2d.hpp>
#include <godot_cpp/classes/physics_shape_query_parameters2d.hpp>
#include <godot_cpp/classes/rectangle_shape2d.hpp>
#include "platformer_astar2d.h"

namespace godot {

class PathAction : public godot::RefCounted{
    GDCLASS(PathAction, godot::RefCounted)

protected:
    static void _bind_methods() {
        ClassDB::bind_method(D_METHOD("get_from"), &PathAction::get_from);
        ClassDB::bind_method(D_METHOD("set_from", "from"), &PathAction::set_from); 
        ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "from"), "set_from", "get_from");
        ClassDB::bind_method(D_METHOD("get_is_jump"), &PathAction::get_is_jump);
        ClassDB::bind_method(D_METHOD("set_is_jump", "is_jump"), &PathAction::set_is_jump);
        ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_jump"), "set_is_jump", "get_is_jump");
        ClassDB::bind_method(D_METHOD("get_jump_velocity"), &PathAction::get_jump_velocity);
        ClassDB::bind_method(D_METHOD("set_jump_velocity", "jump_velocity"), &PathAction::set_jump_velocity);
        ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "jump_velocity"), "set_jump_velocity", "get_jump_velocity");
        ClassDB::bind_method(D_METHOD("get_to"), &PathAction::get_to);
        ClassDB::bind_method(D_METHOD("set_to", "to"), &PathAction::set_to);
        ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "to"), "set_to", "get_to");
    }
public:
    Vector2 from;
    bool is_jump = false;
    Vector2 jump_velocity;
    Vector2 to;
    
    Vector2 get_from() const { return from; }
    void set_from(const Vector2 &p_from) { from = p_from; }
    bool get_is_jump() const { return is_jump; }
    void set_is_jump(bool p_is_jump) { is_jump = p_is_jump; }
    Vector2 get_jump_velocity() const { return jump_velocity; }
    void set_jump_velocity(const Vector2 &p_jump_velocity) { jump_velocity = p_jump_velocity; }
    Vector2 get_to() const { return to; }
    void set_to(const Vector2 &p_to) { to = p_to; }
};

class Pathfinder : public godot::Node {
	GDCLASS(Pathfinder, godot::Node)

private:
    Ref<PlatformerAStar2D> graph;
    NodePath debug_draw;
    NodePath nav_region;

    std::vector<PackedVector2Array> islands;
    // the walkable parts, subdivided according to surface_subdivision_distance
    std::vector<PackedVector2Array> island_surface_pts;
    std::vector<PackedInt64Array> island_surface_node_ids;

    // Parameters for changing graph generation
    float max_walkable_surface_angle = 55.0f;
    // The max allowable distance between points on the surface of islands, otherwise subdivided
    float max_surface_subdivision_distance = 5.0f;

    // The max allowable linear distance from points to jump between, quick filter without physics-based jump test simulation
    float max_jump_distance = 400.0f;

    Vector2 agent_size = {15, 15};

    float collision_tests_per_second = 25.0f; 
    Vector2 acceleration = {0.0, 1.0};

    bool to_generate_graph = true; 
protected:
	static void _bind_methods();

public:
    void set_debug_draw(const NodePath &p_debug_draw);
    NodePath get_debug_draw() const;

    void set_nav_region(const NodePath &p_nav_region);
    NodePath get_nav_region() const;

	Pathfinder();
	~Pathfinder();

	void _process(double delta) override;

    godot::TypedArray<PathAction> find_path(Vector2 from, Vector2 to);
    void generate_graph();

    float try_find_unobstructed_jump(Vector2& out_velocity, Vector2 from, Vector2 to, int test_intervals);
};


}

#endif // PATHFINDER_H