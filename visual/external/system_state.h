//
// Created by Daniel Schatz on 03.11.25.
//

#ifndef PLUSSIM_SYSTEM_STATE_H
#define PLUSSIM_SYSTEM_STATE_H


namespace atg_scs {
    class SystemState {
    public:
        SystemState();
        ~SystemState();

        void copy(const SystemState *state);
        void resize(int bodyCount, int constraintCount);
        void destroy();

        void localToWorld(double x, double y, double z, double *x_t, double *y_t, double *z_t, int body);
        void velocityAtPoint(double x, double y, double z, double *v_x, double *v_y, double *v_z, int body);
        void applyForce(double x_l, double y_l, double z_l, double f_x, double f_y, double f_z, int body);

        int *indexMap;

        // Angular acceleration, velocity, and orientation (Euler angles or quaternions)
        double *a_theta_x;
        double *a_theta_y;
        double *a_theta_z;
        double *v_theta_x;
        double *v_theta_y;
        double *v_theta_z;
        double *theta_x;
        double *theta_y;
        double *theta_z;

        // Linear acceleration
        double *a_x;
        double *a_y;
        double *a_z;

        // Linear velocity
        double *v_x;
        double *v_y;
        double *v_z;

        // Position
        double *p_x;
        double *p_y;
        double *p_z;

        // Forces
        double *f_x;
        double *f_y;
        double *f_z;

        // Torques
        double *t_x;
        double *t_y;
        double *t_z;

        // Constraint positions (local coordinates)
        double *r_x;
        double *r_y;
        double *r_z;
        double *r_t_x;
        double *r_t_y;
        double *r_t_z;

        double *m;

        int n;
        int n_c;
        double dt;
    };
} /* namespace atg_scs */


#endif //PLUSSIM_SYSTEM_STATE_H