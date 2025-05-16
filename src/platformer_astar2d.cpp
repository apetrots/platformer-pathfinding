#include "platformer_astar2d.h"

using namespace godot;

void PlatformerAStar2D::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("add_jump_node", "from", "to", "jump_velocity"), &PlatformerAStar2D::add_jump_node);
    ClassDB::bind_method(D_METHOD("is_jump_node", "node"), &PlatformerAStar2D::is_jump_node);
    ClassDB::bind_method(D_METHOD("get_jump_velocity", "jump_node_id"), &PlatformerAStar2D::get_jump_velocity);
}

// Override the _compute_cost function
float PlatformerAStar2D::_compute_cost(int64_t from_id, int64_t to_id) const
{
    // if going from a jump node to a normal node, add to cost.
    if (is_jump_node(from_id))
    {
        return 1.0f/jump_node_weight_scale;
    }

    return 0.0f;
}


float PlatformerAStar2D::_estimate_cost(int64_t from_id, int64_t end_id) const
{
    // Custom cost computation logic for platformer
    return 0.0f; // Example: constant cost for all edges
}


// Add a jump node
int64_t PlatformerAStar2D::add_jump_node(int64_t from, int64_t to, Vector2 jump_velocity) {
    int64_t jump_node = get_point_count();

    this->add_point(jump_node, get_point_position(from), jump_node_weight_scale);
    this->connect_points(from, jump_node, false);
    this->connect_points(jump_node, to, false);

    jump_velocities[jump_node] = jump_velocity;

    return jump_node;
}

// Check if a node is a jump node
bool PlatformerAStar2D::is_jump_node(int64_t node) const {
    // Do performant, quick check first before making sure...
    if (this->get_point_weight_scale(node) != jump_node_weight_scale)
        return false;

    return jump_velocities.find(node) != jump_velocities.end();
}

// Get the jump velocity for a jump node
Vector2 PlatformerAStar2D::get_jump_velocity(int64_t jump_node_id) {
    auto it = jump_velocities.find(jump_node_id);
    if (it != jump_velocities.end()) {
        return it->second;
    }
    return Vector2(); // Return a default Vector2 if not found
}