#include <iostream>
#include "kinematics_base.hpp"

int main() {
    auto kin = ELITE::loadKinematicsSolver("libelite_kdl_kinematics_example.so");
    if (!kin) {
        std::cerr << "Failed to load kinematics plugin\n";
        return 1;
    }

    std::vector<double> a{0.0, 0.0, -0.427, -0.3905, 0.0, 0.0};
    std::vector<double> d{0.1625, 0.0, 0.0, 0.14750, 0.0965, 0.095};
    std::vector<double> alpha{0.0, 1.5707963, 0.0, 0.0, 1.5707963, -1.5707963};

    kin->configDH(a, d, alpha);  // DH参数配置

    // 示例位姿输入q
    ELITE::JOINTS q{0.223, -1.628, 0.796, -1.084, 1.452, 0.978};
    ELITE::MAT4X4 T{};
    std::array<double, 6> tcp{0, 0, 0, 0, 0, 0};

    kin->forward(q, T, tcp);
    std::cout << "Forward kinematics (TCP 4x4 matrix):\n";
    std::cout.setf(std::ios::fixed);
    std::cout.precision(6);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::cout << T[r][c] << (c == 3 ? "" : "\t");
        }
        std::cout << "\n";
    }
    ELITE::JOINTS q_near{0.6, -1.1, 0.0, 0.0, 1.9, 3.0};
    ELITE::JOINTS q_out{};
    int ret = kin->inverse(T, q_near, q_out, tcp);
    std::cout << "IK ret = " << ret << "\n";
    if (ret == 0) {
        std::cout << "Inverse kinematics joints (rad): ";
        for (std::size_t i = 0; i < q_out.size(); ++i) {
            std::cout << q_out[i] << (i + 1 == q_out.size() ? "\n" : ", ");
        }
    }
    return 0;
}
