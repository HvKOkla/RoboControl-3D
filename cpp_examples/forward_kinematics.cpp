#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

// Simple 4x4 Matrix structure for DH parameters
struct Matrix4x4 {
    double data[4][4];
};

// Compute the standard DH transformation matrix
Matrix4x4 getDHMatrix(double a, double alpha, double d, double theta) {
    Matrix4x4 m;
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);

    m.data[0][0] = ct;      m.data[0][1] = -st * ca; m.data[0][2] = st * sa;  m.data[0][3] = a * ct;
    m.data[1][0] = st;      m.data[1][1] = ct * ca;  m.data[1][2] = -ct * sa; m.data[1][3] = a * st;
    m.data[2][0] = 0;       m.data[2][1] = sa;       m.data[2][2] = ca;       m.data[2][3] = d;
    m.data[3][0] = 0;       m.data[3][1] = 0;        m.data[3][2] = 0;        m.data[3][3] = 1.0;
    return m;
}

// Multiply two 4x4 matrices
Matrix4x4 multiply(const Matrix4x4& A, const Matrix4x4& B) {
    Matrix4x4 C;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            C.data[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                C.data[i][j] += A.data[i][k] * B.data[k][j];
            }
        }
    }
    return C;
}

int main() {
    std::cout << "--- RoboControl 3D: C++ Forward Kinematics ---" << std::endl;

    // DH Parameters for a 3-DOF arm (angles in radians)
    // Format: a, alpha, d, theta_offset
    std::vector<std::vector<double>> dh = {
        {0.0, M_PI/2, 15.0, 0.0},
        {30.0, 0.0, 0.0, 0.0},
        {25.0, 0.0, 0.0, 0.0}
    };

    // Joint angles in degrees (matching the web simulator test)
    std::vector<double> jointAngles = {45.0, -30.0, 20.0};

    // Identity matrix
    Matrix4x4 T = {{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}}};

    for (size_t i = 0; i < dh.size(); ++i) {
        double thetaRad = jointAngles[i] * M_PI / 180.0 + dh[i][3];
        Matrix4x4 linkMatrix = getDHMatrix(dh[i][0], dh[i][1], dh[i][2], thetaRad);
        T = multiply(T, linkMatrix);
    }

    // Extract XYZ position from the final transformation matrix
    double x = T.data[0][3];
    double y = T.data[1][3];
    double z = T.data[2][3];

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "End-Effector Position:\n";
    std::cout << "X: " << x << " cm\n";
    std::cout << "Y: " << y << " cm\n";
    std::cout << "Z: " << z << " cm\n";

    return 0;
}