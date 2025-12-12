#include "elite_kinematics/kdl_kinematics_example.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

namespace ELITE {

namespace {
void KDLJointsToJoints(const KDL::JntArray &kdl_joints, JOINTS &joints) {
    joints.resize(kdl_joints.rows());
    for (unsigned int i = 0; i < kdl_joints.rows(); i++) {
        joints[i] = kdl_joints(i);
    }
}

KDL::JntArray JointsToKDLJoints(const JOINTS &joints, int joint_count) {
    KDL::JntArray kdl_joints(joint_count);
    for (int i = 0; i < joint_count; i++) {
        kdl_joints(i) = (i < static_cast<int>(joints.size())) ? joints[i] : 0.0;
    }
    return kdl_joints;
}

void KDLFrameToMat4x4(const KDL::Frame &frame, MAT4X4 mat4x4) {
    mat4x4[0][0] = frame.M.data[0];
    mat4x4[0][1] = frame.M.data[1];
    mat4x4[0][2] = frame.M.data[2];
    mat4x4[0][3] = frame.p.data[0];

    mat4x4[1][0] = frame.M.data[3];
    mat4x4[1][1] = frame.M.data[4];
    mat4x4[1][2] = frame.M.data[5];
    mat4x4[1][3] = frame.p.data[1];

    mat4x4[2][0] = frame.M.data[6];
    mat4x4[2][1] = frame.M.data[7];
    mat4x4[2][2] = frame.M.data[8];
    mat4x4[2][3] = frame.p.data[2];

    mat4x4[3][0] = 0;
    mat4x4[3][1] = 0;
    mat4x4[3][2] = 0;
    mat4x4[3][3] = 1;
}

KDL::Frame Mat4x4ToKDLFrame(const MAT4X4 mat4x4) {
    KDL::Frame frame = KDL::Frame::Identity();
    frame.M.data[0] = mat4x4[0][0];
    frame.M.data[1] = mat4x4[0][1];
    frame.M.data[2] = mat4x4[0][2];
    frame.p.data[0] = mat4x4[0][3];

    frame.M.data[3] = mat4x4[1][0];
    frame.M.data[4] = mat4x4[1][1];
    frame.M.data[5] = mat4x4[1][2];
    frame.p.data[1] = mat4x4[1][3];

    frame.M.data[6] = mat4x4[2][0];
    frame.M.data[7] = mat4x4[2][1];
    frame.M.data[8] = mat4x4[2][2];
    frame.p.data[2] = mat4x4[2][3];
    return frame;
}
}  // namespace

Kinematics::Kinematics() : _jointCount(0) {}

Kinematics::~Kinematics() = default;

void Kinematics::forward(const JOINTS &joints, MAT4X4 mat, const std::array<double, 6> &tcp_offset_xyzRPY) {
    if (!_fkSolver || static_cast<int>(joints.size()) < _jointCount) {
        std::cerr << "Please set Kinematics config first by updateKinConfig\n";
        return;
    }
    KDL::JntArray kdl_joints = JointsToKDLJoints(joints, _jointCount);
    KDL::Frame end_effector_frame;
    _fkSolver->JntToCart(kdl_joints, end_effector_frame);

    KDL::Rotation rot = KDL::Rotation::EulerZYX(tcp_offset_xyzRPY[5], tcp_offset_xyzRPY[4], tcp_offset_xyzRPY[3]);

    KDL::Frame tcp_offset(rot, KDL::Vector(tcp_offset_xyzRPY[0], tcp_offset_xyzRPY[1], tcp_offset_xyzRPY[2]));
    KDL::Frame tcp_frame = end_effector_frame * tcp_offset;

    KDLFrameToMat4x4(tcp_frame, mat);
}

int Kinematics::inverse(MAT4X4 mat, const JOINTS &joints_near, JOINTS &joints, const std::array<double, 6> &tcp_offset_xyzRPY) {
    if (!_ikSolver || static_cast<int>(joints_near.size()) < _jointCount) {
        return -1;
    }
    KDL::Rotation rot = KDL::Rotation::RPY(tcp_offset_xyzRPY[3], tcp_offset_xyzRPY[4], tcp_offset_xyzRPY[5]);
    KDL::Frame tcp_offset(rot, KDL::Vector(tcp_offset_xyzRPY[0], tcp_offset_xyzRPY[1], tcp_offset_xyzRPY[2]));

    KDL::JntArray kdl_joints_near = JointsToKDLJoints(joints_near, _jointCount);
    KDL::Frame frame = Mat4x4ToKDLFrame(mat);
    frame = frame * tcp_offset.Inverse();
    KDL::JntArray ret_joints(_jointCount);
    int result = _ikSolver->CartToJnt(kdl_joints_near, frame, ret_joints);
    if (result == KDL::SolverI::E_NOERROR) {
        KDLJointsToJoints(ret_joints, joints);
    }
    return (result == KDL::SolverI::E_NOERROR) ? 0 : -1;
}

bool Kinematics::isDefault() const { return false; }

void Kinematics::configDH(const JOINTS &dh_a, const JOINTS &dh_d, const JOINTS &dh_alpha) {
    std::lock_guard<std::mutex> lock(_dataMutex);

    if (!(dh_a.size() == dh_d.size() && dh_d.size() == dh_alpha.size()) || dh_a.empty()) {
        std::cerr << "Invalid DH sizes\n";
        return;
    }

    _ikSolver.reset();
    _fkSolver.reset();
    _jacSolver.reset();
    _robotChain.reset();

    _robotChain = std::make_unique<KDL::Chain>();

    _jointCount = static_cast<int>(dh_a.size());
    _dhA = dh_a;
    _dhD = dh_d;
    _dhAlpha = dh_alpha;

    for (int i = 0; i < _jointCount; i++) {
        KDL::Frame rot =
            KDL::Frame(KDL::Rotation(KDL::Vector(1, 0, 0), KDL::Vector(0, std::cos(_dhAlpha[i]), std::sin(_dhAlpha[i])),
                                     KDL::Vector(0, -std::sin(_dhAlpha[i]), std::cos(_dhAlpha[i]))));
        KDL::Frame trans = KDL::Frame(KDL::Vector(_dhA[i], 0, _dhD[i]));
        _robotChain->addSegment(KDL::Segment("Link" + std::to_string(i), KDL::Joint(KDL::Joint::None), rot * trans));
        _robotChain->addSegment(KDL::Segment("Link" + std::to_string(i), KDL::Joint(KDL::Joint::RotZ)));
    }

    _ikSolver = std::make_unique<KDL::ChainIkSolverPos_LMA>(*(_robotChain), 1E-10, 500);
    _fkSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(*(_robotChain));
    _jacSolver = std::make_unique<KDL::ChainJntToJacSolver>(*(_robotChain));
}

}  // namespace ELITE

// =============== 导出为 dlopen 插件 ===============
extern "C" ELITE::KinematicsBase *createKinematicsPlugin() { return new ELITE::Kinematics(); }
