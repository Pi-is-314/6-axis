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

struct IKResult {
    bool   ok;
    double q1, q2, q3;   // degrees
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
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    Mat3 R = {{
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp,     cp * sr,                cp * cr}
    }};
    return R;
}

IKResult solveWristIK(double wx, double wy, double wz,
                      bool flipBase, bool flipElbow) {
    const double L1 = 17.5, L2 = 27.4954, e = 0.45, z0 = 11.56;
    const double R2D = 180.0 / M_PI;
    IKResult r = {false, 0, 0, 0};

    double r2 = wx*wx + wy*wy - e*e;
    if (r2 < 0) return r;                       // too close to the base axis
    double u = std::sqrt(r2);
    if (flipBase) u = -u;

    double b  = wz - z0;
    double c3 = (u*u + b*b - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    if (c3 > 1.0 || c3 < -1.0) return r;        // out of reach

    double s3 = std::sqrt(1.0 - c3*c3);
    if (flipElbow) s3 = -s3;

    r.q3 = std::atan2(s3, c3) * R2D;
    r.q2 = (std::atan2(u, b) - std::atan2(L2 * s3, L1 + L2 * c3)) * R2D;
    r.q1 = (std::atan2(wx, -wy) - std::atan2(e, u)) * R2D;
    r.ok = true;
    return r;
}

struct OriResult { bool ok; double q4, q5, q6; };   // degrees

// R03 and Rt are 3x3 (use the upper-left of your Mat4s)
OriResult solveWristOrientation(const double R03[3][3], const double Rt[3][3], bool flip,
                                double currentQ4 = 0.0) {
    const double R2D = 180.0 / M_PI, D2R = M_PI / 180.0;
    double m[3][3];                                  // m = R03^T * Rt
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            m[i][j] = 0;
            for (int k = 0; k < 3; k++) m[i][j] += R03[k][i] * Rt[k][j];
        }

    double sb = std::sqrt(m[0][2]*m[0][2] + m[1][2]*m[1][2]);
    double beta, alpha, gamma;

    if (sb > 1e-6) {
        double sgn = flip ? -1.0 : 1.0;
        beta  = std::atan2(sgn * sb, m[2][2]);
        alpha = std::atan2(sgn * m[1][2],  sgn * m[0][2]);
        gamma = std::atan2(sgn * m[2][1], -sgn * m[2][0]);
    } else {                                         // singular: theta5 = 0 or 180
        beta  = (m[2][2] > 0) ? 0.0 : M_PI;
        alpha = (currentQ4 + 90.0) * D2R;            // keep theta4 where it is
        gamma = (beta == 0.0) ? std::atan2(m[1][0], m[0][0]) - alpha
                              : alpha - std::atan2(-m[1][0], -m[0][0]);
    }
    return { true, alpha * R2D - 90.0, beta * R2D, gamma * R2D - 90.0 };
}