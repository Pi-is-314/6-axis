// dh_kinematics.cpp
// Stores standard-DH parameters for an N-joint arm, builds each link's 4x4
// transform, and chains them for forward kinematics.
// No external libraries and no heap allocation, so it ports easily to an ESP32.
//
// Build (PC):  g++ -std=c++17 -o dh dh_kinematics.cpp && ./dh
 
#include <array>
#include <cmath>
#include <cstdio>
 
constexpr int N = 6;                       // number of joints
constexpr double DEG2RAD = M_PI / 180.0;
 
using Mat4 = std::array<std::array<double, 4>, 4>;
 
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
 
class Arm {
public:
    // Store one row of the DH table
    void setRow(int i, double theta_deg, double d, double a, double alpha_deg,
                double offset = 0.0, JointType type = JointType::Revolute) {
        rows_[i] = {theta_deg * DEG2RAD, d, a, alpha_deg * DEG2RAD,
                    type == JointType::Revolute ? offset * DEG2RAD : offset, type};
    }
 
    // Set joint variables q (degrees for revolute, length units for prismatic)
    void setJoints(const std::array<double, N>& q) {
        for (int i = 0; i < N; ++i)
            q_[i] = (rows_[i].type == JointType::Revolute) ? q[i] * DEG2RAD : q[i];
    }
 
    // Transform for link i alone (frame i-1 -> frame i)
    Mat4 linkTransform(int i) const {
        const DHRow& r = rows_[i];
        double theta = r.theta, d = r.d;
        if (r.type == JointType::Revolute) theta += q_[i] + r.offset;
        else                               d     += q_[i] + r.offset;
        return dhMatrix(theta, d, r.a, r.alpha);
    }
 
    // Base -> end effector
    Mat4 forward() const {
        Mat4 T = identity();
        for (int i = 0; i < N; ++i) T = multiply(T, linkTransform(i));
        return T;
    }
 
    // Base -> frame k (k = 1..N)
    Mat4 frame(int k) const {
        Mat4 T = identity();
        for (int i = 0; i < k; ++i) T = multiply(T, linkTransform(i));
        return T;
    }
 
    void printTable() const {
        std::printf("DH table (angles in deg)\n");
        std::printf(" i   theta      d        a      alpha    offset   type\n");
        for (int i = 0; i < N; ++i) {
            const DHRow& r = rows_[i];
            std::printf("%2d %8.2f %8.2f %8.2f %8.2f %8.2f   %s\n", i + 1,
                        r.theta / DEG2RAD, r.d, r.a, r.alpha / DEG2RAD,
                        r.type == JointType::Revolute ? r.offset / DEG2RAD : r.offset,
                        r.type == JointType::Revolute ? "R" : "P");
        }
    }
 
private:
    std::array<DHRow, N> rows_{};
    std::array<double, N> q_{};
};
 
void printMat(const char* name, const Mat4& M) {
    std::printf("%s\n", name);
    for (const auto& row : M)
        std::printf("  [% 9.4f % 9.4f % 9.4f % 9.4f]\n", row[0], row[1], row[2], row[3]);
}
 
int main() {
    Arm arm;
 
    // ---- EDIT THIS TABLE to match your arm ----------------------------------
    //         i theta   d      a      alpha  offset
    arm.setRow(0, 0,   11.56,  
        10.95,   90.0,    0.0);
    arm.setRow(1, 0,    17.5,  10.5,    0.0,  -90.0);
    arm.setRow(2, 0,    17.5,  10.5,   90.0,    0.0);
    arm.setRow(3, 0,    20.8,   0.0,  -90.0,    0.0);
    arm.setRow(4, 0,   2.414,   0.0,   90.0,    90.0);
    arm.setRow(5, 0,     5.5,   0.0,    0.0,    90.0);
    // --------------------------------------------------------------------------
 
    arm.printTable();
 
    arm.setJoints({0, 0, 0, 0, 0, 0});     // joint values in degrees
    std::printf("\n");
    for (int i = 0; i < N; ++i) {
        char label[32];
        std::snprintf(label, sizeof(label), "T(%d-1 -> %d):", i + 1, i + 1);
        printMat(label, arm.linkTransform(i));
    }
 
    Mat4 T = arm.forward();
    std::printf("\n");
    printMat("Base -> end effector:", T);
    std::printf("\nEnd effector position: x=%.3f y=%.3f z=%.3f\n", T[0][3], T[1][3], T[2][3]);
    return 0;
}