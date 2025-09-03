#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sstream>  // for std::ostringstream

namespace uts {

inline Eigen::Matrix3d skew(const Eigen::Vector3d& p){
  Eigen::Matrix3d S;
  S <<    0, -p.z(),  p.y(),
       p.z(),     0, -p.x(),
      -p.y(),  p.x(),     0;
  return S;
}

// Adjoint for [v; w] ordering:
// Ad_T = [[ R, [p]^ R ],
//         [ 0,     R   ]]
inline Eigen::Matrix<double,6,6> adjoint(const Eigen::Matrix4d& T){
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  Eigen::Vector3d p = T.block<3,1>(0,3);
  Eigen::Matrix<double,6,6> Ad = Eigen::Matrix<double,6,6>::Zero();
  Ad.block<3,3>(0,0) = R;
  Ad.block<3,3>(0,3) = skew(p) * R;
  Ad.block<3,3>(3,3) = R;
  return Ad;
}

inline std::string matrixToInitializer(const Eigen::MatrixXd& M, int precision=8){
  std::ostringstream oss;
  oss.setf(std::ios::fixed); oss.precision(precision);
  for(int r=0;r<M.rows();++r){
    for(int c=0;c<M.cols();++c){
      oss << M(r,c);
      if(!(r==M.rows()-1 && c==M.cols()-1)) oss << ", ";
    }
  }
  return oss.str();
}

// Decompose T into position + quaternion (WXYZ order).
inline void TToPosQuatWXYZ(const Eigen::Matrix4d& T,
                           Eigen::Vector3d& p_out,
                           Eigen::Vector4d& q_wxyz_out){
  p_out = T.block<3,1>(0,3);
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  Eigen::Quaterniond q(R); // (w, x, y, z)
  if (q.norm() == 0.0) {
    q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  } else {
    q.normalize();
  }
  // Export as wxyz
  q_wxyz_out = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

// position + quaterion into T
inline Eigen::Matrix4d posQuatWXYZToT(const Eigen::Vector3d& p, const Eigen::Vector4d& q_wxyz){
  Eigen::Quaterniond q(q_wxyz(0), q_wxyz(1), q_wxyz(2), q_wxyz(3)); // (w,x,y,z)
  if (q.norm() == 0.0) q = Eigen::Quaterniond(1.0,0.0,0.0,0.0);
  else q.normalize();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = q.toRotationMatrix();
  T.block<3,1>(0,3) = p;
  return T;
}

// Handy YAML list formatter
inline std::string vecToYamlList(const Eigen::VectorXd& v, int precision=10){
  std::ostringstream oss; oss.setf(std::ios::fixed); oss.precision(precision);
  oss << "[";
  for (int i=0;i<v.size();++i){ oss << v(i); if(i+1<v.size()) oss << ", "; }
  oss << "]";
  return oss.str();
}



} // namespace uts