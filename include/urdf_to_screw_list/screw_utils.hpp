#pragma once
#include <Eigen/Dense>

namespace uts {

inline Eigen::Matrix3d skew(const Eigen::Vector3d& p){
  Eigen::Matrix3d S;
  S <<    0, -p.z(),  p.y(),
       p.z(),     0, -p.x(),
      -p.y(),  p.x(),     0;
  return S;
}

inline Eigen::Matrix<double,6,6> adjoint(const Eigen::Matrix4d& T){
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  Eigen::Vector3d p = T.block<3,1>(0,3);

  // For twists in [v; w] ordering, the adjoint is:
  // Ad_{T} = [[ R,  [p]^ R ],
  //           [ 0,      R   ]]
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

} // namespace uts
