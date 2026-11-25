//
// Created by Daniel Schatz on 03.11.25.
//

#ifndef PLUSSIM_SYSTEM_STATE_H
#define PLUSSIM_SYSTEM_STATE_H
#include <vector>


class SystemState {
    public:
        void copy(const SystemState *state);
        void resize(int bodyCount, int constraintCount);

        void localToWorld(double x, double y, double z, double *x_t, double *y_t, double *z_t, int body);
        void velocityAtPoint(double x, double y, double z, double *v_x, double *v_y, double *v_z, int body);
        void applyForce(double x_l, double y_l, double z_l, double f_x, double f_y, double f_z, int body);

        std::vector<int> indexMap;

        // Angular acceleration, velocity, and orientation (Euler angles or quaternions)
        std::vector<double> a_theta_x, a_theta_y, a_theta_z;
        std::vector<double> v_theta_x, v_theta_y, v_theta_z;
        std::vector<double> theta_x, theta_y, theta_z;

        // Linear acceleration, velocity, position
        std::vector<double> a_x, a_y, a_z;
        std::vector<double> v_x, v_y, v_z;
        std::vector<double> p_x, p_y, p_z;

        // Forces and torques
        std::vector<double> f_x, f_y, f_z;
        std::vector<double> t_x, t_y, t_z;

        // Constraint positions (local coordinates)
        std::vector<double> r_x, r_y, r_z;
        std::vector<double> r_t_x, r_t_y, r_t_z;

        std::vector<double> m;

        int n = 0;
        int n_c = 0;
        double dt = 0.0;
    };



#endif //PLUSSIM_SYSTEM_STATE_H