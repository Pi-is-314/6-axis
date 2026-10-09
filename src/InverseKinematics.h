#include <array>
#include <cmath>
#include <cstdio>
 
constexpr int N = 6;                       // number of joints
constexpr double DEG2RAD = M_PI / 180.0;
 
using Mat4 = std::array<std::array<double, 4>, 4>;
using Mat3 = std::array<std::array<double, 3>, 3>;

 
enum class JointType { Revolute, Prismatic };

struct DHRow {
    double theta;      // rad  (joint variable if revolute)
    double d;          // length units (joint variable if prismatic)
    double a;          // link length
    double alpha;      // rad  link twist
    double offset;     // rad (revolute) or length (prismatic) added to the joint variable
    JointType type;
};

 
Mat4 identity() {
    Mat4 m{};
    for (int i = 0; i < 4; ++i) m[i][i] = 1.0;
    return m;
}
 
Mat4 multiply(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}
 
// Standard DH transform from frame i-1 to frame i
Mat4 dhMatrix(double theta, double d, double a, double alpha) {
    const double ct = std::cos(theta), st = std::sin(theta);
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    Mat4 T = {{
        {ct, -st * ca,  st * sa, a * ct},
        {st,  ct * ca, -ct * sa, a * st},
        {0.0,      sa,       ca,      d},
        {0.0,     0.0,      0.0,    1.0}
    }};
    return T;
}

Mat3 convertRollPitchYawToMatrix(double roll, double pitch, double yaw) {
    Mat3 R;
    R[0][0] = 
    return R;
}