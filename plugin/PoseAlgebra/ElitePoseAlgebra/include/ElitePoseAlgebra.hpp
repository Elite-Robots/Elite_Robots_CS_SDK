#ifndef __ELITE_POSE_ALGEBRA_HPP__
#define __ELITE_POSE_ALGEBRA_HPP__

#include <Elite/ClassLoader.hpp>
#include <Elite/PoseAlgebraBase.hpp>

namespace ELITE {

class ElitePoseAlgebra : public PoseAlgebraBase {
   public:
    ElitePoseAlgebra() = default;
    ~ElitePoseAlgebra() override = default;

    ELITE_EXPORT bool inverse(const PoseMatrix& pose, PoseMatrix& inverse_pose, PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool inverse(const vector6d_t& pose, vector6d_t& inverse_pose, PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool multiply(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                               PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool multiply(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                               PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool add(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                          PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool add(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                          PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool subtract(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                               PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool subtract(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                               PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool vectorToMatrix(const vector6d_t& pose_vector, PoseMatrix& pose_matrix,
                                     PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool matrixToVector(const PoseMatrix& pose_matrix, vector6d_t& pose_vector,
                                     PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool distance(const PoseMatrix& pose_a, const PoseMatrix& pose_b, PoseDistance& out_distance,
                               PoseAlgebraResult& result) const override;

    ELITE_EXPORT bool distance(const vector6d_t& pose_a, const vector6d_t& pose_b, PoseDistance& out_distance,
                               PoseAlgebraResult& result) const override;

      ELITE_EXPORT bool worldToLocal(const PoseMatrix& world_ref_pose, const PoseMatrix& world_pose,
                              PoseMatrix& local_pose, PoseAlgebraResult& result) const override;

      ELITE_EXPORT bool worldToLocal(const vector6d_t& world_ref_pose, const vector6d_t& world_pose,
                              vector6d_t& local_pose, PoseAlgebraResult& result) const override;

      ELITE_EXPORT bool localToWorld(const PoseMatrix& world_ref_pose, const PoseMatrix& local_pose,
                              PoseMatrix& world_pose, PoseAlgebraResult& result) const override;

      ELITE_EXPORT bool localToWorld(const vector6d_t& world_ref_pose, const vector6d_t& local_pose,
                              vector6d_t& world_pose, PoseAlgebraResult& result) const override;
};

}  // namespace ELITE

#endif