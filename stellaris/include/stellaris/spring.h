//
// Created by Daniel Schatz on 25.11.26.
//

#ifndef PLUSSIM_SPRING_H
#define PLUSSIM_SPRING_H
#include "stellaris/force_generator.h"

class Spring : public ForceGenerator {
public:
    Spring(int body, double anchor_x, double anchor_y, double anchor_z, double spring_k, double damping, double rest_length) : m_body(body), m_anchor_x(anchor_x), m_anchor_y(anchor_y), m_anchor_z(anchor_z),
        m_spring_k(spring_k), m_damping(damping), m_rest_length(rest_length) {}
    void apply(SystemState &state) override;
    void set(double anchor_x, double anchor_y, double anchor_z,
         double spring_k, double damping, double rest_length) {
        m_anchor_x = anchor_x;
        m_anchor_y = anchor_y;
        m_anchor_z = anchor_z;
        m_spring_k = spring_k;
        m_damping = damping;
        m_rest_length = rest_length;
    }
private:
    int m_body;
    double m_anchor_x, m_anchor_y, m_anchor_z;
    double m_spring_k;
    double m_damping;
    double m_rest_length;
};
#endif //PLUSSIM_SPRING_H
