#include <hex_kinematic/kinematic.hpp>
#include <Eigen/Cholesky>
#include <tinyxml2.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace hex_kinematic {
namespace {
double wrap(double q) { return q - 2 * pi * std::floor((q + pi) / (2 * pi)); }
double f32(double x) { return static_cast<float>(x); }
double nearest(const std::vector<double>& candidates, double current) {
  return *std::min_element(candidates.begin(), candidates.end(), [current](double a, double b) {
    return std::abs(a - current) < std::abs(b - current);
  });
}
Vec3 xyz(const tinyxml2::XMLElement* e, const char* attribute) {
  Vec3 v = Vec3::Zero();
  if (e && e->Attribute(attribute)) {
    std::istringstream s(e->Attribute(attribute)); s >> v.x() >> v.y() >> v.z();
  }
  return v;
}
Mat3 rz(double q) {
  const double c = std::cos(q), s = std::sin(q);
  Mat3 r; r << c,-s,0, s,c,0, 0,0,1; return r;
}
}  // namespace

Kinematic::Kinematic(const std::string& urdf, double a, double b, double c, double bx,
                     double by, double threshold)
    : l1(a), l2(b), l3(c), singularity_threshold(threshold) {
  // 与 Python 中 float32 保存的几何、限位保持一致，避免阈值附近分支差异。
  rest_limits << f32(-140*pi/180), f32(135*pi/180), f32(-150*pi/180), f32(165*pi/180);
  for (int i=0; i<6; ++i) {
    const auto& name = leg_names[i];
    const bool left = name[0]=='l', back=name[1]=='b', middle=name[1]=='m';
    leg_base.row(i) << f32(left ? -bx : bx), f32(middle ? 0 : (back ? -by : by)), 0;
    const bool negative = name=="lf" || name=="rb", positive=name=="rf" || name=="lb";
    thigh_limits.row(i) << f32((negative ? -92 : -40)*pi/180), f32((positive ? 92 : 40)*pi/180);
  }
  if (!urdf.empty()) LoadMassModel(urdf);
}

void Kinematic::LoadMassModel(const std::string& filename) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Cannot read URDF: " + filename);
  const auto* root=doc.FirstChildElement("robot");
  if (!root) throw std::runtime_error("URDF has no robot element");
  std::map<std::string,const tinyxml2::XMLElement*> links, joints;
  for (auto* e=root->FirstChildElement("link"); e; e=e->NextSiblingElement("link")) links[e->Attribute("name")]=e;
  for (auto* e=root->FirstChildElement("joint"); e; e=e->NextSiblingElement("joint")) joints[e->Attribute("name")]=e;
  auto inertial=[&](const std::string& name, double& mass, Vec3& com) {
    const auto* node=links.at(name)->FirstChildElement("inertial");
    mass=node ? node->FirstChildElement("mass")->DoubleAttribute("value") : 0;
    com=node ? xyz(node->FirstChildElement("origin"),"xyz") : Vec3::Zero();
  };
  inertial("body",body_mass_,body_com_);
  const std::array<std::string,8> parts{"thigh","knee","ankle","foot","ball1","ball2","suck","toe"};
  const std::array<Vec3,7> positions{Vec3(.072,0,0),Vec3(.13,0,0),Vec3(.17,0,0),
      Vec3(.001,0,0),Vec3::Zero(),Vec3::Zero(),Vec3(0,0,.026)};
  const std::array<Vec3,7> rotations{Vec3(pi/2,0,0),Vec3::Zero(),Vec3::Zero(),
      Vec3(0,pi/2,0),Vec3::Zero(),Vec3::Zero(),Vec3::Zero()};
  const std::array<Vec3,7> axes{Vec3(0,0,1),Vec3(0,0,1),Vec3(0,0,1),
      Vec3(1,0,0),Vec3(0,1,0),Vec3(0,0,1),Vec3::Zero()};
  for (int i=0; i<6; ++i) {
    for (int j=0; j<8; ++j) {
      inertial("l_"+leg_names[i]+"_"+parts[j],masses_[i][j],link_com_[i][j]);
      const auto* joint=joints.at("j_"+leg_names[i]+"_"+parts[j]);
      const auto* origin=joint->FirstChildElement("origin");
      const Vec3 axis=xyz(joint->FirstChildElement("axis"),"xyz");
      if ((j==0 && (std::string(joint->Attribute("type"))!="revolute" || (axis-Vec3(0,0,1)).norm()>1e-10)) ||
          (j>0 && (std::string(joint->Attribute("type"))!=(j==7 ? "fixed" : "revolute") ||
          (xyz(origin,"xyz")-positions[j-1]).norm()>1e-10 ||
          (xyz(origin,"rpy")-rotations[j-1]).norm()>1e-5 || (axis-axes[j-1]).norm()>1e-10)))
        throw std::runtime_error("Unsupported direct-FK geometry: "+leg_names[i]+"/"+parts[j]);
    }
    leg_masses[i]=0;
    for (double mass:masses_[i]) leg_masses[i]+=mass;
  }
  total_mass=body_mass_+leg_masses.sum();
}

Vec3 Kinematic::ForwardKinReturn(const Vec3& q) const {
  const double r=l1+l2*std::cos(q[1])+l3*std::cos(q[1]+q[2]);
  return {r*std::cos(q[0]),r*std::sin(q[0]),l2*std::sin(q[1])+l3*std::sin(q[1]+q[2])};
}
Rows3 Kinematic::ForwardKin(const Rows3& q) const {
  Rows3 p(q.rows(),3);
  for (int i=0; i<q.rows(); ++i) p.row(i)=ForwardKinReturn(q.row(i).transpose());
  return p;
}
Vec3 Kinematic::ForwardKinKnee(double q) const { return {l1*std::cos(q),l1*std::sin(q),0}; }
Vec3 Kinematic::ForwardKinAnkle(const Eigen::Vector2d& q) const {
  return {(l1+l2*std::cos(q[1]))*std::cos(q[0]),(l1+l2*std::cos(q[1]))*std::sin(q[0]),l2*std::sin(q[1])};
}
Vec3 Kinematic::Get_X_B3(const Vec3& q) const {
  return {std::cos(q[1]+q[2])*std::cos(q[0]),std::cos(q[1]+q[2])*std::sin(q[0]),std::sin(q[1]+q[2])};
}
Vec3 Kinematic::Get_Z_B3(const Vec3& q) const { return {std::sin(q[0]),-std::cos(q[0]),0}; }
Mat3 Kinematic::Jacobian(const Vec3& q) const {
  const double c=std::cos(q[0]),s=std::sin(q[0]),z=l2*std::sin(q[1])+l3*std::sin(q[1]+q[2]);
  const double r=l1+l2*std::cos(q[1])+l3*std::cos(q[1]+q[2]),d=l3*std::cos(q[1]+q[2]),e=l3*std::sin(q[1]+q[2]);
  Mat3 J; J << -r*s,-z*c,-e*c, r*c,-z*s,-e*s, 0,r-l1,d; return J;
}
Hessian3 Kinematic::Hessian(const Vec3& q) const {
  const double c=std::cos(q[0]),s=std::sin(q[0]),A=l2*std::sin(q[1])+l3*std::sin(q[1]+q[2]);
  const double B=l2*std::cos(q[1])+l3*std::cos(q[1]+q[2]),C=l3*std::sin(q[1]+q[2]),D=l3*std::cos(q[1]+q[2]);
  Hessian3 h;
  h[0] << -(l1+B)*c,A*s,C*s, A*s,-B*c,-D*c, C*s,-D*c,-D*c;
  h[1] << -(l1+B)*s,-A*c,-C*c, -A*c,-B*s,-D*s, -C*c,-D*s,-D*s;
  h[2] << 0,0,0, 0,-A,-C, 0,-C,-C;
  return h;
}
Vec3 Kinematic::JacobianDotQdot(const Vec3& q,const Vec3& dq) const {
  const auto h=Hessian(q); Vec3 result;
  for (int i=0;i<3;++i) result[i]=dq.dot(h[i]*dq);
  return result;
}
bool Kinematic::SingularityZ(const Vec3& q) const {
  return std::abs(l1+l2*std::cos(q[1])+l3*std::cos(q[1]+q[2]))<singularity_threshold;
}
Mat3 Kinematic::DampInvJac(const Vec3& q) const { return DampInvJac(q,SingularityZ(q)); }
Mat3 Kinematic::DampInvJac(const Vec3& q,bool singular) const {
  const Mat3 J=Jacobian(q);
  Mat3 inv=(J*J.transpose()+1e-4*Mat3::Identity()).ldlt().solve(J).transpose();
  if (singular) inv.row(0).setZero();
  return inv;
}
Vec3 Kinematic::CF_Estimate(const Vec3& tau,const Vec3& q) const { return CF_Estimate(tau,q,SingularityZ(q)); }
Vec3 Kinematic::CF_Estimate(const Vec3& tau,const Vec3& q,bool singular) const {
  const Mat3 J=Jacobian(q);
  // 足端对环境的力：F = J (J^T J + λI)^-1 τ；奇异区屏蔽 thigh 力矩。
  Vec3 torque=tau; if (singular) torque[0]=0;
  return J*(J.transpose()*J+1e-5*Mat3::Identity()).ldlt().solve(torque);
}
Vec3 Kinematic::LegVectorToR(const Vec3& v,int leg) const {
  Vec3 result=v; if (leg<3) result.head<2>()*=-1; return result;
}
Vec3 Kinematic::B2R(const Vec3& p,int leg) const { return LegVectorToR(p,leg)+leg_base.row(leg).transpose(); }
Vec3 Kinematic::R2B(const Vec3& p,int leg) const { return LegVectorToR(p-leg_base.row(leg).transpose(),leg); }

FKResult Kinematic::ForwardKinAll(const Angles& q) const {
  if (total_mass<=0) throw std::runtime_error("ForwardKinAll requires a URDF mass model");
  FKResult result; Vec3 weighted=body_mass_*body_com_;
  Mat3 X,Y; X << 1,0,0, 0,0,-1, 0,1,0; Y << 0,0,1, 0,1,0, -1,0,0;
  for (int i=0;i<6;++i) {
    // 每条腿只求一次三角函数；同一变换复用于全部质量连杆。
    const Mat3 Z1=rz(q(i,0)),Z2=rz(q(i,1)),Z3=rz(q(i,2)),Z4=rz(q(i,3));
    const Vec3 ex(1,0,0),base=Z1*(.072*ex+X*Z2*(.13*ex+Z3*(.17*ex)));
    const Mat3 R=Z1*X*Z2*Z3*Z4;
    const auto& c=link_com_[i];
    const std::array<Vec3,8> points{Z1*c[0],Z1*(.072*ex+X*Z2*c[1]),
      Z1*(.072*ex+X*Z2*(.13*ex+Z3*c[2])),base+R*c[3],base+R*(.001*ex),
      base+R*(.001*ex),base+R*(.001*ex+Y*c[6]),base+R*(.027*ex)};
    Vec3 com=Vec3::Zero();
    for (int j=0;j<8;++j) com+=masses_[i][j]*points[j];
    com/=leg_masses[i];
    result.feet.row(i)=base; result.leg_com.row(i)=com;
    weighted+=leg_masses[i]*B2R(com,i);
  }
  result.com=weighted/total_mass; return result;
}

Vec3 Kinematic::InverseKin1(const Vec3& p,Vec3 q) const {
  for (int i=0;i<1000;++i) {
    const Vec3 error=p-ForwardKinReturn(q);
    if (error.norm()<.005) break;
    q+=.01*DampInvJac(q)*error;
  }
  return q;
}
Vec3 Kinematic::InverseKin2(const Vec3& p,const Vec3& current) const {
  double q1=std::atan2(p.y(),p.x()); if (q1>pi/2) q1-=pi; else if (q1<-pi/2) q1+=pi;
  const double r=std::cos(q1)*p.x()+std::sin(q1)*p.y()-l1;
  const double angle=std::acos(std::clamp((r*r+p.z()*p.z()-l2*l2-l3*l3)/(2*l2*l3),-1.,1.));
  const double q3=nearest({angle,-angle},current[2]);
  const double back=std::atan2(l3*std::sin(q3),l3*std::cos(q3)+l2);
  const double front=std::asin(p.z()/std::hypot(l3*std::cos(q3)+l2,l3*std::sin(q3)));
  return {q1,nearest({front-back,-front+pi-back,-front-pi-back},current[1]),q3};
}
IKResult Kinematic::InverseKin2Branches(const Vec3& p) const {
  IKResult result;
  for (auto& q:result.q) q.setConstant(std::numeric_limits<double>::quiet_NaN());
  double q1=std::atan2(p.y(),p.x()); if (q1>pi/2) q1-=pi; else if (q1<-pi/2) q1+=pi;
  const double r=std::cos(q1)*p.x()+std::sin(q1)*p.y()-l1;
  const double c=(r*r+p.z()*p.z()-l2*l2-l3*l3)/(2*l2*l3);
  if (c < -1-1e-9 || c > 1+1e-9) return result;
  const double angle=std::acos(std::clamp(c,-1.,1.));
  for (int branch=0;branch<2;++branch) {
    const double q3=branch ? -angle : angle;
    if (q3<f32(-31*pi/36) || q3>f32(7*pi/9)) continue;
    const double back=std::atan2(l3*std::sin(q3),l3*std::cos(q3)+l2);
    const double denom=std::hypot(l3*std::cos(q3)+l2,l3*std::sin(q3));
    if (denom<1e-12) continue;
    const double front=std::asin(std::clamp(p.z()/denom,-1.,1.));
    for (double q2:{front-back,-front+pi-back,-front-pi-back}) {
      const Vec3 q(q1,q2,q3);
      if (q2>=f32(-2*pi/3) && q2<=f32(25*pi/36) && (ForwardKinReturn(q)-p).norm()<1e-6) {
        result.q[branch]=q.unaryExpr([](double x){ return wrap(x); });
        result.valid[branch]=true; break;
      }
    }
  }
  return result;
}
std::vector<IKResult> Kinematic::InverseKin2MultiBatch(const Rows3& p) const {
  std::vector<IKResult> results; results.reserve(p.rows());
  for (int i=0;i<p.rows();++i) results.push_back(InverseKin2Branches(p.row(i).transpose()));
  return results;
}
Rows3 Kinematic::InverseKin2Multi(const Vec3& p) const {
  const auto branches=InverseKin2Branches(p); std::vector<Vec3> values;
  for (int i=0;i<2;++i) if (branches.valid[i]) {
    Vec3 v=branches.q[i].unaryExpr([](double x){return std::round(x*1e10)/1e10;});
    if (values.empty() || (values.back()-v).norm()>1e-10) values.push_back(v);
  }
  std::sort(values.begin(),values.end(),[](const Vec3& a,const Vec3& b){
    for (int j=0;j<3;++j) if (a[j]!=b[j]) return a[j]<b[j]; return false;
  });
  Rows3 result(values.size(),3); for (size_t i=0;i<values.size();++i) result.row(i)=values[i]; return result;
}
Angles Kinematic::InitialJoints(const Rows3& values,bool feet,const std::array<int,6>& branches) const {
  if (values.rows()!=6 || !values.allFinite()) throw std::invalid_argument("Initial joints/feet require six finite rows");
  Angles q;
  for (int i=0;i<6;++i) {
    Vec3 motor=values.row(i).transpose();
    if (branches[i]!=0 && branches[i]!=1) throw std::invalid_argument("IK branch must be 0 or 1");
    if (feet) {
      const auto solutions=InverseKin2Branches(motor);
      if (!solutions.valid[branches[i]]) throw std::invalid_argument("Invalid initial IK branch: "+leg_names[i]);
      motor=solutions.q[branches[i]];
    } else if ((motor[2]>1e-8 && branches[i]!=0) || (motor[2]<-1e-8 && branches[i]!=1))
      throw std::invalid_argument("Initial ankle sign disagrees with IK branch");
    if (motor[0]<thigh_limits(i,0)-1e-6 || motor[0]>thigh_limits(i,1)+1e-6 ||
        motor[1]<rest_limits(0,0)-1e-6 || motor[1]>rest_limits(0,1)+1e-6 ||
        motor[2]<rest_limits(1,0)-1e-6 || motor[2]>rest_limits(1,1)+1e-6)
      throw std::invalid_argument("Initial joint exceeds limits: "+leg_names[i]);
    q.row(i).head<3>()=motor; q(i,3)=-motor[1]-motor[2]-pi/2;
  }
  return q;
}
}  // namespace hex_kinematic
