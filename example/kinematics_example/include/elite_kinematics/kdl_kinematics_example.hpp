#pragma once

#include <array>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include "kinematics_base.hpp"
namespace ELITE {

class Kinematics : public KinematicsBase {
   public:
    Kinematics();
    ~Kinematics() override;

    // ==== KinematicsBase 接口实现 ====
    std::string name() const override { return "KDL_Kinematics_example"; }
    std::size_t dof() const override { return _jointCount; }
    bool isDefault() const override;  // 是否为默认求解器

    void configDH(const JOINTS &dh_a, const JOINTS &dh_d, const JOINTS &dh_alpha) override;

    void forward(const JOINTS &joints, MAT4X4 mat, const std::array<double, 6> &tcp_offset_xyzRPY) override;  // mat为输出结果

    int inverse(MAT4X4 mat, const JOINTS &joints_near, JOINTS &joints,
                const std::array<double, 6> &tcp_offset_xyzRPY) override;  // mat为输入目标位姿

   private:
    int _jointCount;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> _ikSolver;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> _fkSolver;
    std::unique_ptr<KDL::ChainJntToJacSolver> _jacSolver;
    std::unique_ptr<KDL::Chain> _robotChain;

    JOINTS _dhA;
    JOINTS _dhD;
    JOINTS _dhAlpha;

    std::mutex _dataMutex;
};

}  // namespace ELITE
