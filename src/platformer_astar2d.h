#ifndef PLATFORMER_ASTAR2D_H
#define PLATFORMER_ASTAR2D_H

#include <godot_cpp/classes/tile_map_layer.hpp>
#include <godot_cpp/classes/polygon2d.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/classes/a_star2d.hpp>
#include <godot_cpp/classes/navigation_region2d.hpp>
#include <godot_cpp/classes/navigation_polygon.hpp>
#include <godot_cpp/classes/engine.hpp>

namespace godot {

struct JumpInfo {
    real_t jump_duration;
    Vector2 from;
    Vector2 to;
};

class PlatformerAStar2D : public AStar2D {
    GDCLASS(PlatformerAStar2D, AStar2D)
    
    float jump_node_weight_scale = 100.0f;
    std::unordered_map<int64_t, JumpInfo> jumps;
protected:
    static void _bind_methods();
public:
    float _compute_cost(int64_t from_id, int64_t to_id) const override;

    float _estimate_cost(int64_t from_id, int64_t end_id) const override;

    int64_t add_jump_node(int64_t from, int64_t to, real_t jump_duration);

    bool is_jump_node(int64_t node) const;

    real_t get_jump_duration(int64_t jump_node_id);
};

}

#endif // PLATFORMER_ASTAR2D_H