#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <godot_cpp/classes/polygon2d.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/classes/a_star2d.hpp>
#include <godot_cpp/classes/navigation_region2d.hpp>
#include <godot_cpp/classes/navigation_polygon.hpp>
#include <godot_cpp/classes/engine.hpp>
#include "platformer_astar2d.h"

namespace godot {

class Pathfinder : public godot::Node {
	GDCLASS(Pathfinder, godot::Node)

private:
    Ref<PlatformerAStar2D> graph;
    NodePath debug_draw;
    NodePath nav_region;

    std::vector<PackedVector2Array> islands;
    // the walkable parts, subdivided according to surface_subdivision_distance
    std::vector<PackedVector2Array> island_surface_pts;

    // Parameters for changing graph generation
    float max_walkable_surface_angle = 55.0f;
    // The max allowable distance between points on the surface of islands, otherwise subdivided
    float max_surface_subdivision_distance = 5.0f;

    // The max allowable linear distance from points to jump between, quick filter without physics-based jump test simulation
    float max_jump_distance = 400.0f;
protected:
	static void _bind_methods();

public:
    void set_debug_draw(const NodePath &p_debug_draw);
    NodePath get_debug_draw() const;

    void set_nav_region(const NodePath &p_nav_region);
    NodePath get_nav_region() const;

	Pathfinder();
	~Pathfinder();

    void _ready() override;
	void _process(double delta) override;

    void find_path(Vector2 from, Vector2 to);
    void generate_graph();
};

}

#endif // PATHFINDER_H