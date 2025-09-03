#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <optional>

#include "urdf_to_screw_list/screw_utils.hpp"

using std::placeholders::_1;

struct ScrewList {
  Eigen::MatrixXd S; // 6 x n (space screws)
  Eigen::Matrix4d M; // home pose (base->ee at q=0)
  std::vector<std::string> joint_names;
};

static Eigen::Matrix4d kdlFrameToEig(const KDL::Frame& F){
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for(int r=0;r<3;++r)
    for(int c=0;c<3;++c)
      T(r,c) = F.M(r,c);
  T(0,3)=F.p.x(); T(1,3)=F.p.y(); T(2,3)=F.p.z();
  return T;
}

static std::string readFileToString(const std::string& path){
  std::ifstream ifs(path);
  if(!ifs) throw std::runtime_error("Cannot open file: " + path);
  std::stringstream buffer; buffer << ifs.rdbuf();
  return buffer.str();
}

static ScrewList make_space_screws_from_urdf_string(
    const std::string& urdf_xml,
    const std::string& base_link,
    const std::string& ee_link)
{
  KDL::Tree tree;
  if(!kdl_parser::treeFromString(urdf_xml, tree))
    throw std::runtime_error("Failed to parse URDF to KDL tree.");

  KDL::Chain chain;
  if(!tree.getChain(base_link, ee_link, chain))
    throw std::runtime_error("Failed to extract KDL chain from " + base_link + " to " + ee_link + ".");

  const int segs = chain.getNrOfSegments();

  // Running transform from base to "previous tip" (start at base)
  KDL::Frame T_base_prev = KDL::Frame::Identity();

  Eigen::MatrixXd S(6,0);
  std::vector<std::string> joint_names;

  for(int i=0;i<segs;++i){
    const auto& seg = chain.getSegment(i);
    const auto& jnt = seg.getJoint();

    // Transform from previous tip to current joint frame at q=0
    KDL::Frame F_prev_to_joint = jnt.pose(0.0);
    KDL::Frame T_base_joint = T_base_prev * F_prev_to_joint;

    // If movable joint, form screw about space frame (base)
    if(jnt.getType() != KDL::Joint::None && jnt.getType() != KDL::Joint::Fixed){
      // Axis in joint frame:
      KDL::Vector a_j = jnt.JointAxis();

      // Rotate axis into base frame and position of joint in base frame
      KDL::Vector w_kdl = T_base_joint.M * a_j;
      KDL::Vector q_kdl = T_base_joint.p;

      Eigen::Vector3d w(w_kdl.x(), w_kdl.y(), w_kdl.z());
      Eigen::Vector3d q(q_kdl.x(), q_kdl.y(), q_kdl.z());
      Eigen::Vector3d v;

      // Revolute types in KDL:
      if (jnt.getType() == KDL::Joint::RotAxis ||
          jnt.getType() == KDL::Joint::RotX   ||
          jnt.getType() == KDL::Joint::RotY   ||
          jnt.getType() == KDL::Joint::RotZ)
      {
        v = - w.cross(q); // revolute: S = [-w x q; w]
      } else {
        // Prismatic types (TransAxis/X/Y/Z): set w = 0, v = axis in space
        v = w;
        w.setZero();
      }

      Eigen::Matrix<double,6,1> Si; Si << v, w; // (linear on top, angular below)
      S.conservativeResize(6, S.cols()+1);
      S.col(S.cols()-1) = Si;

      joint_names.push_back(jnt.getName());
    }

    // advance to current tip for next loop: pose(0) = joint.pose(0) * frame_to_tip
    T_base_prev = T_base_prev * seg.pose(0.0);
  }

  ScrewList out;
  out.S = S;
  out.M = kdlFrameToEig(T_base_prev); // base->ee at q=0
  out.joint_names = joint_names;
  return out;
}

class UTSNode : public rclcpp::Node {
public:
  UTSNode() : Node("urdf_to_screw_list_node") {
    // Parameters
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("ee_link", "tool0");
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("urdf_path", "");      // optional fallback
    this->declare_parameter<std::string>("output_path", "");     // optional file
    this->declare_parameter<std::string>("output_format", "yaml"); // yaml|txt|cpp
    this->declare_parameter<bool>("use_body_frame", false);      // false = space S, true = body B
    this->declare_parameter<bool>("verbose", true);
    this->declare_parameter<bool>("home_pose_as_pos_quat", true);

    run_once();
  }

private:
  void run_once(){
    const auto base_link       = this->get_parameter("base_link").as_string();
    const auto ee_link         = this->get_parameter("ee_link").as_string();
    const auto output_path     = this->get_parameter("output_path").as_string();
    const auto out_fmt         = this->get_parameter("output_format").as_string();
    const bool use_body        = this->get_parameter("use_body_frame").as_bool();
    const bool verbose         = this->get_parameter("verbose").as_bool();
    const bool home_pose_as_pos_quat   = this->get_parameter("home_pose_as_pos_quat").as_bool();

    std::string urdf_xml = this->get_parameter("robot_description").as_string();
    if(urdf_xml.empty()){
      auto urdf_path = this->get_parameter("urdf_path").as_string();
      if(urdf_path.empty()){
        RCLCPP_ERROR(get_logger(), "Neither 'robot_description' nor 'urdf_path' provided.");
        rclcpp::shutdown(); return;
      }
      urdf_xml = readFileToString(urdf_path);
    }

    ScrewList space = make_space_screws_from_urdf_string(urdf_xml, base_link, ee_link);
    Eigen::MatrixXd S = space.S;
    Eigen::Matrix4d M = space.M;

    Eigen::MatrixXd S_or_B = S;
    std::string frame_label = "space";
    if(use_body){
      Eigen::Matrix<double,6,6> AdMinv = uts::adjoint(M.inverse());
      S_or_B = AdMinv * S;
      frame_label = "body";
    }

    // ---- Print to console ----
    const int n = static_cast<int>(S_or_B.cols());
    std::ostringstream oss;
    oss.setf(std::ios::fixed); oss.precision(8);

    if(verbose){
      RCLCPP_INFO(get_logger(), "Chain %s -> %s with %d joints.", base_link.c_str(), ee_link.c_str(), n);
      std::string names;
      for(size_t i=0;i<space.joint_names.size();++i){
        names += space.joint_names[i];
        if(i+1<space.joint_names.size()) names += ", ";
      }
      RCLCPP_INFO(get_logger(), "Joint order: [%s]", names.c_str());
    }

    // // YAML-ish string (emits M as position + quaternion xyzw)
    // std::string out_text;
    // {
    //   std::ostringstream y;
    //   y.setf(std::ios::fixed); y.precision(10);

    //   // Add chain endpoints first
    //   y << "base_link: " << base_link << "\n";
    //   y << "ee_link: " << ee_link << "\n";
    //   y << "screw_representation (space or body): " << frame_label << "\n";
    //   y << "joint_names: [";

    //   for(size_t i=0;i<space.joint_names.size();++i){
    //     y << space.joint_names[i];
    //     if(i+1<space.joint_names.size()) y << ", ";
    //   }
    //   y << "]\n";
    //   y << "num_joints: " << n << "\n";
    //   y << "screw_list: # S = [v; w] = [-w x q; w]\n";
    //   for(int j=0;j<n;++j){
    //     Eigen::VectorXd col = S_or_B.col(j);
    //     y << "  - [" << col(0) << ", " << col(1) << ", " << col(2)
    //       << ", "   << col(3) << ", " << col(4) << ", " << col(5) << "]\n";
    //   }

    //   if (home_pose_as_pos_quat) {
    //     Eigen::Vector3d Mp;
    //     Eigen::Vector4d Mq_wxyz;
    //     uts::TToPosQuatWXYZ(M, Mp, Mq_wxyz);
    //     y << "M_position: " << uts::vecToYamlList(Mp) << "\n";
    //     y << "M_quaternion_wxyz: " << uts::vecToYamlList(Mq_wxyz) << "\n";
    //   } else {
    //     y << "M_matrix:\n";
    //     for(int r=0;r<4;++r){
    //       y << "  - [" << M(r,0) << ", " << M(r,1) << ", " << M(r,2) << ", " << M(r,3) << "]\n";
    //     }
    //   }

    //   out_text = y.str();
    // }


    // YAML-ish string (emits M as position + quaternion xyzw)
    std::string out_text;
    {
      std::ostringstream y;
      y.setf(std::ios::fixed); 
      y.precision(10);

      // Add chain endpoints first
      y << "base_link: " << base_link << "\n";
      y << "ee_link: " << ee_link << "\n";
      y << "screw_representation: " << frame_label << "  # space or body\n";

      // joint_names as list
      y << "joint_names: [";
      for (size_t i = 0; i < space.joint_names.size(); ++i) {
        y << space.joint_names[i];
        if (i + 1 < space.joint_names.size()) y << ", ";
      }
      y << "]\n";

      y << "num_joints: " << n << "\n";

      // screw_list as dictionary mapping joint_name -> [w, v]
      y << "screw_list:\n";
      for (int j = 0; j < n; ++j) {
        Eigen::VectorXd col = S_or_B.col(j);
        const std::string& jname = space.joint_names[j];
        y << "  " << jname << ": [" 
          << col(0) << ", " << col(1) << ", " << col(2) << ", "
          << col(3) << ", " << col(4) << ", " << col(5) << "]\n";
      }

      if (home_pose_as_pos_quat) {
        Eigen::Vector3d Mp;
        Eigen::Vector4d Mq_wxyz;
        uts::TToPosQuatWXYZ(M, Mp, Mq_wxyz);
        y << "M_position: " << uts::vecToYamlList(Mp) << "\n";
        y << "M_quaternion_wxyz: " << uts::vecToYamlList(Mq_wxyz) << "\n";
      } else {
        y << "M_matrix:\n";
        for (int r = 0; r < 4; ++r) {
          y << "  - [" << M(r,0) << ", " << M(r,1) << ", " << M(r,2) << ", " << M(r,3) << "]\n";
        }
      }

      out_text = y.str();
    }


    RCLCPP_INFO(get_logger(), "\n%s", out_text.c_str());


    if(!output_path.empty()){
      std::ofstream ofs(output_path);
      if(!ofs){
        RCLCPP_ERROR(get_logger(), "Failed to open output_path: %s", output_path.c_str());
      }else{
        ofs << out_text;
        RCLCPP_INFO(get_logger(), "Wrote screw list to %s", output_path.c_str());
      }
    }

  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<UTSNode>();  // constructor runs once and finishes
  }  // ensure node is destroyed before shutdown
  rclcpp::shutdown();
  return 0;
}
