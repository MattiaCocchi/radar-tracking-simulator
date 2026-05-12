#ifndef ENTITY_H
#define ENTITY_H

struct State {
    double x  = 0.0;
    double y  = 0.0;
    double vx = 0.0;
    double vy = 0.0;
};

class Entity {
public:
    Entity(double x, double y, double vx, double vy);

    void update(double dt);

    State getState() const;

private:
    State e_state;
};

#endif 