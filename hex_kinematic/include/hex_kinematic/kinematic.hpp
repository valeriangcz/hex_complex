#pragma once

#include <Eigen/Core>
#include <array>
#include <string>
#include <vector>

namespace hex_kinematic {
inline constexpr double pi = 3.14159265358979323846;
inline const std::array<std::string, 6> leg_names{"lb", "lf", "lm", "rb", "rf", "rm"};
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Rows3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using JointMatrix = Eigen::Matrix<double, 6, 3, Eigen::RowMajor>;
using Angles = Eigen::Matrix<double, 6, 4, Eigen::RowMajor>;
using Ratios = Eigen::Matrix<double, 6, 1>;
using Flags = std::array<bool, 6>;
using Hessian3 = std::array<Mat3, 3>;  // xyz 各分量对三个关节的二阶导数

struct FKResult {
  JointMatrix feet, leg_com;
  Vec3 com;
};
struct IKResult {
  std::array<Vec3, 2> q;
  std::array<bool, 2> valid{false, false};
};

// 数值库不依赖 ROS。角度使用 rad，长度使用 m；点按行保存。
class Kinematic {
 public:
  explicit Kinematic(const std::string& urdf = "", double l1 = .072, double l2 = .13,
                     double l3 = .17, double bx = .1, double by = .22,
                     double singularity_threshold = .02);
  Vec3 ForwardKinReturn(const Vec3& q) const;
  Rows3 ForwardKin(const Rows3& q) const;
  Vec3 ForwardKinKnee(double q1) const;
  Vec3 ForwardKinAnkle(const Eigen::Vector2d& q) const;
  Vec3 Get_X_B3(const Vec3& q) const;
  Vec3 Get_Z_B3(const Vec3& q) const;
  Mat3 Jacobian(const Vec3& q) const;
  Hessian3 Hessian(const Vec3& q) const;
  Vec3 JacobianDotQdot(const Vec3& q, const Vec3& dq) const;
  bool SingularityZ(const Vec3& q) const;
  Mat3 DampInvJac(const Vec3& q) const;
  Mat3 DampInvJac(const Vec3& q, bool singular) const;
  Vec3 CF_Estimate(const Vec3& tau, const Vec3& q) const;
  Vec3 CF_Estimate(const Vec3& tau, const Vec3& q, bool singular) const;
  Vec3 B2R(const Vec3& p, int leg) const;
  Vec3 R2B(const Vec3& p, int leg) const;
  Vec3 LegVectorToR(const Vec3& v, int leg) const;
  Vec3 RVectorToLeg(const Vec3& v, int leg) const { return LegVectorToR(v, leg); }
  FKResult ForwardKinAll(const Angles& q) const;
  Vec3 InverseKin1(const Vec3& p, Vec3 q) const;
  Vec3 InverseKin2(const Vec3& p, const Vec3& current) const;
  Rows3 InverseKin2Multi(const Vec3& p) const;
  IKResult InverseKin2Branches(const Vec3& p) const;
  std::vector<IKResult> InverseKin2MultiBatch(const Rows3& p) const;
  Angles InitialJoints(const Rows3& values, bool feet, const std::array<int, 6>& branches) const;

  double l1, l2, l3, singularity_threshold, total_mass = 0;
  JointMatrix leg_base;
  Eigen::Matrix<double, 6, 2> thigh_limits;
  Eigen::Matrix<double, 2, 2> rest_limits;
  Ratios leg_masses = Ratios::Zero();

 private:
  void LoadMassModel(const std::string& filename);
  std::array<std::array<Vec3, 8>, 6> link_com_;
  std::array<std::array<double, 8>, 6> masses_;
  Vec3 body_com_ = Vec3::Zero();
  double body_mass_ = 0;
};
}  // namespace hex_kinematic
