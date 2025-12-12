#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace ELITE {

using JOINTS = std::vector<double>;
using MAT4X4 = double[4][4];

class KinematicsBase {
   public:
    virtual ~KinematicsBase() = default;

    virtual std::string name() const = 0;
    virtual std::size_t dof() const = 0;
    virtual bool isDefault() const { return false; }

    virtual void configDH(const JOINTS &dh_a, const JOINTS &dh_d,
                          const JOINTS &dh_alpha) = 0;  // 配置DH参数

    virtual void forward(const JOINTS &joints, MAT4X4 mat,
                         const std::array<double, 6> &tcp_offset_xyzRPY) = 0;  // 前向运动学，joints为输入关节角，mat为输出结果

    virtual int inverse(MAT4X4 mat, const JOINTS &joints_near, JOINTS &joints, const std::array<double, 6> &tcp_offset_xyzRPY) = 0;
};  // 逆向运动学，返回0表示成功,mat为输入目标位姿,joints_near为输入附近关节角,
    // joints为输出关节角

using KinematicsBasePtr = std::shared_ptr<KinematicsBase>;
using CreateKinematicsFn = KinematicsBase *(*)();

KinematicsBasePtr loadKinematicsSolver(const std::string &library_path, const std::string &symbol_name = "createKinematicsPlugin");

}  // namespace ELITE
