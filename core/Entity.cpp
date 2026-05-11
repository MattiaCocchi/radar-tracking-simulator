#include "Entity.h"

Entity::Entity(double x, double y, double vx, double vy)
    : e_state{x, y, vx, vy} {}

void Entity::update(double dt) {
    e_state.x += e_state.vx * dt;
    e_state.y += e_state.vy * dt;
}

State Entity::getState() const {
    return e_state;
}