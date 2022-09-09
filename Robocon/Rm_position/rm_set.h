#ifndef RM_SET
#define RM_SET

class Rm_set {
    private:

    public:
        const static int MOTOR_NUM = 4;  // amount of motors supported by 1 can id
        const static int MAX_RPM = 9333;  // max rpm for M3508 P19
        const static int MAX_OUTPUT = 16384; 
        const static int GEAR_RATIO = 19;
        const static int REVOLUTION = 8192;
        const static int NOMINAL_FPS = 1000;
        const static int MAX_ACCEL = 100; //rpm / s
        const static int NOISE_THRESHOLD = 1000;
        
        Rm_set(int id);

        int id;

        int motor_mode[MOTOR_NUM] = {0};
        int target_rpm[MOTOR_NUM] = {0};
        int target_pos[MOTOR_NUM] = {0};
        int rpm[MOTOR_NUM] = {0};
        int pos[MOTOR_NUM] = {0};
        int gearbox_pos[MOTOR_NUM] = {0};
        int can_frame_counter[MOTOR_NUM] = {0};
        int can_frame_prev_time[MOTOR_NUM] = {0};
        int can_fps[MOTOR_NUM] = {0};
        double pid_k_rpm[MOTOR_NUM][3] = {{0.0}};
        double pid_o_rpm[MOTOR_NUM][3] = {{0.0}};
        int rpm_error[MOTOR_NUM] = {0};
        int prev_rpm_error[MOTOR_NUM] = {0};

        int prev_rpm[MOTOR_NUM] = {0};
        int prev_pos[MOTOR_NUM] = {0};
        int prev_time[MOTOR_NUM] = {0};
        int output[MOTOR_NUM] = {0};
        int t = 0;
        int fs_counter;

        // og gearbox counting
        bool cur_low[MOTOR_NUM] = {false};
        bool cur_high[MOTOR_NUM] = {false};
        bool prev_low[MOTOR_NUM] = {false};
        bool prev_high[MOTOR_NUM] = {false};


        unsigned char can_msg[MOTOR_NUM * 2] = {0};


        void set_pid_rpm(int id, double p, double i, double d);
        void set_target_rpm(int id, int rpm);
        void set_target_pos(int id, int pos);
        void run_rpm_pid_cycle(int id);

        void update_motor_status(int id, long micros, int pos, int rpm);
        void reset_gearbox_pos(int id);

};

#endif