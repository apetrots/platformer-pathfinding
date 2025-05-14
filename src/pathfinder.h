#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <godot_cpp/classes/tile_map_layer.hpp>
#include <godot_cpp/classes/polygon2d.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/classes/a_star2d.hpp>
#include <godot_cpp/classes/navigation_region2d.hpp>
#include <godot_cpp/classes/navigation_polygon.hpp>
#include <godot_cpp/classes/engine.hpp>

namespace godot {

class Pathfinder : public godot::Node {
	GDCLASS(Pathfinder, godot::Node)

private:
    Ref<AStar2D> graph;
    NodePath debug_draw;
    NodePath nav_region;

    std::vector<PackedVector2Array> islands;
    std::vector<PackedVector2Array> island_walkable_pts;

    // Parameters for changing graph generation
    float max_walkable_surface_angle = 55.0f;
    // The distance between points at which the edge will be subdivided
    float edge_subdivision_distance = 5.0f;

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