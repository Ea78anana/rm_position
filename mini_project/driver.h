#ifndef CARBASE
#define CARBASE

class Carbase{
    public:
        Carbase(double max_a, double max_v, double max_w, double max_w_a);

        double max_a = 0;
        double max_v = 0;
        double max_w = 0;
        double max_w_a = 0;

        double target_v[2] = {0};
        double current_v[2] = {0};
        double delta_v[2] = {0};
        double current_w = 0;
        double target_w = 0;

        double accel_force[2] = {0};
        double centri_force[2] = {0};

        int last_time = 0;

        void update(double target_v[2], double target_w);
        void update_from_controller(double x, double y, double w);
        void tick(int micros);
};


#endif
