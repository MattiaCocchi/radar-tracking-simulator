#ifndef ENTITY_H
#define ENTITY_H

#include <iostream>

struct State{
    double x, y, vx, vy;
};

class Entity{
    private:
    State e_state;
    public:
    Entity(double x, double y, double vx, double vy);
    void update(double dt);
    State getState() const;
};

#endif