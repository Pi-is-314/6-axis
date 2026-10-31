#include <array>
#include <cmath>
#include <cstdio>
 

 
using Mat4 = std::array<std::array<double, 4>, 4>;
using Mat3 = std::array<std::array<double, 3>, 3>;


Mat3 convertRollPitchYawToMatrix(double roll, double pitch, double yaw, Mat3 RInitial) {
    
    Mat3 Rx = {{
      {1, 0, 0},
      {0, std::cos(roll), -std::sin(roll)},
      {0, std::sin(roll), std::cos(roll)}
  }};

    Mat3 Ry = {{
      {std::cos(pitch), 0, std::sin(pitch)},
      {0, 1, 0},
      {-std::sin(pitch), 0, std::cos(pitch)}
  }};

    Mat3 Rz = {{
      {std::cos(yaw), -std::sin(yaw), 0},
      {std::sin(yaw), std::cos(yaw), 0},
      {0, 0, 1}
  }};
    Mat3 Rfinal{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Rfinal[i][j] = Rz[i][0] * (Ry[0][0] * Rx[0][j] + Ry[0][1] * Rx[1][j] + Ry[0][2] * Rx[2][j]) +
                           Rz[i][1] * (Ry[1][0] * Rx[0][j] + Ry[1][1] * Rx[1][j] + Ry[1][2] * Rx[2][j]) +
                           Rz[i][2] * (Ry[2][0] * Rx[0][j] + Ry[2][1] * Rx[1][j] + Ry[2][2] * Rx[2][j]);
    
    // Multiply Rfinal with RInitial
    Mat3 Rtotal{};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                Rtotal[i][j] += Rfinal[i][k] * RInitial[k][j];

    return Rtotal;
}








