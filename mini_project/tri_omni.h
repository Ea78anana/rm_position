#ifndef TRI_OMNI
#define TRI_OMNI

#define DEGREE_2_RADIAN 0.0174532925f
#define M_PI 3.14159265358979323846f
#define SQRT3_2 0.866025403

using namespace std;

class Tri_omni{
  private:
    const static int MAX_RPM = 9333;
    const static int NUM_OF_WHEEL = 3;

    double wheel_radius, carbase_radius;

    double carbase_matrix[NUM_OF_WHEEL][3];

  public:
    Tri_omni(double, double);

    void getMovement(int *, double, double, double);
};

#endif
