#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "robot_math_utils/robot_math_utils_v1_15.hpp"
using RM = RMUtils;

// ---------- small helpers ----------
static Matrix4d kdlFrameToEig(const KDL::Frame& F){
  Matrix4d T = Matrix4d::Identity();
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

static std::string vecToYamlList(const VectorXd& v, int precision=10){
  std::ostringstream oss; oss.setf(std::ios::fixed); oss.precision(precision);
  oss << "[";
  for (int i=0;i<v.size();++i){ oss << v(i); if(i+1<v.size()) oss << ", "; }
  oss << "]";
  return oss.str();
}

static std::string writeJointLimits(const ScrewList& screws, int precision = 10) {
  std::ostringstream y;
  y.setf(std::ios::fixed);
  y.precision(precision);

  y << "joint_limits:\n";

  const auto& names = screws.joint_names;
  const auto& JL    = screws.joint_limits; // n×4: [lower, upper, velocity, effort]
  const int n = static_cast<int>(names.size());

  for (int i = 0; i < n; ++i) {
    const std::string& jname = names[i];

    // Guard for size; if missing, emit empty
    if (i >= JL.rows() || JL.cols() < 4) {
      y << "  " << jname << ": {}\n";
      continue;
    }

    const double ll  = JL(i,0);
    const double ul  = JL(i,1);
    const double vel = JL(i,2);
    const double eff = JL(i,3);

    // Heuristic: if [ll,ul]=[0,0] but (vel or eff) present, treat as continuous
    const bool approx_zero_llul = (std::abs(ll) < 1e-12 && std::abs(ul) < 1e-12);
    const bool has_dyn = (std::abs(vel) > 0.0 || std::abs(eff) > 0.0);

    if (approx_zero_llul && has_dyn) {
      y << "  " << jname << ": { type: continuous";
      if (std::abs(vel) > 0.0) y << ", velocity: " << vel;
      if (std::abs(eff) > 0.0) y << ", effort: "   << eff;
      y << " }\n";
    } else if (std::abs(ll) + std::abs(ul) + std::abs(vel) + std::abs(eff) > 0.0) {
      y << "  " << jname << ": { lower: "   << ll
        << ", upper: "     << ul
        << ", velocity: "  << vel
        << ", effort: "    << eff << " }\n";
    } else {
      y << "  " << jname << ": {}\n";
    }
  }
  return y.str();
}

static std::string writeHomePose(const ScrewList& screws, const bool& home_pose_as_pos_quat, int precision = 10) {
  std::ostringstream y;
  y.setf(std::ios::fixed);
  y.precision(precision);

  if (home_pose_as_pos_quat) {
    Vector3d Mp = screws.M.pos;
    Vector4d Mq_wxyz;
    Mq_wxyz << screws.M.quat.w(), screws.M.quat.x(), screws.M.quat.y(), screws.M.quat.z();
    y << "M_position: " << vecToYamlList(Mp) << "\n";
    y << "M_quaternion_wxyz: " << vecToYamlList(Mq_wxyz) << "\n";
  } else {
    Matrix4d M = RM::PosQuat2TMat(screws.M);
    y << "M_matrix:\n";
    for (int r = 0; r < 4; ++r) {
      y << "  - [" << M(r,0) << ", " << M(r,1) << ", " << M(r,2) << ", " << M(r,3) << "]\n";
    }
  }

  return y.str();
}


// ---------- core: build SPACE screws + metadata from URDF ----------
static ScrewList make_screw_list_from_urdf_string(
    const std::string& urdf_xml,
    const std::string& base_link,
    const std::string& ee_link,
    const bool& use_body)
{
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml);
  KDL::Tree tree;
  if(!kdl_parser::treeFromString(urdf_xml, tree))
    throw std::runtime_error("Failed to parse URDF to KDL tree.");

  KDL::Chain chain;
  if(!tree.getChain(base_link, ee_link, chain))
    throw std::runtime_error("Failed to extract KDL chain from " + base_link + " to " + ee_link + ".");

  const int segs = chain.getNrOfSegments();

  // Running transform from base to "previous tip" (start at base)
  KDL::Frame T_base_prev = KDL::Frame::Identity();

  // Generate space screw list
  MatrixXd S(6,0);
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

      Vector3d w(w_kdl.x(), w_kdl.y(), w_kdl.z());
      Vector3d q(q_kdl.x(), q_kdl.y(), q_kdl.z());
      Vector3d v;

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

      Vector6d Si; 
      Si << v, w; // (linear on top, angular below)
      S.conservativeResize(6, S.cols()+1);
      S.col(S.cols()-1) = Si;

      joint_names.push_back(jnt.getName());
    }

    // advance to current tip for next loop: pose(0) = joint.pose(0) * frame_to_tip
    T_base_prev = T_base_prev * seg.pose(0.0);
  }

  // Build joint limits matrix (n×4: [lower, upper, velocity, effort])
  const int n = static_cast<int>(joint_names.size());
  MatrixXd jl = MatrixXd::Zero(n, 4);
  for (int j = 0; j < n; ++j) {
    const std::string& jname = joint_names[j];
    urdf::JointConstSharedPtr jp = urdf_model ? urdf_model->getJoint(jname) : urdf::JointConstSharedPtr();
    if (jp && jp->limits && jp->type != urdf::Joint::CONTINUOUS) {
      jl(j,0) = jp->limits->lower;
      jl(j,1) = jp->limits->upper;
      jl(j,2) = jp->limits->velocity;
      jl(j,3) = jp->limits->effort;
    } else {
      jl.row(j).setZero(); // continuous or no limits → zeros
    }
  }

  // Home pose
  Matrix4d M_b_e = kdlFrameToEig(T_base_prev);

  // Select screw representation (space or body)
  MatrixXd S_or_B = S; // space-screw by default; body-screw if use_body is true
  std::string frame_label = "space";
  if(use_body){
    Matrix6d AdMinv = RM::Adjoint( M_b_e.inverse() );
    S_or_B = AdMinv * S;
    frame_label = "body";
  }

  // Construct ScrewList (SPACE screws by default) + metadata
  ScrewList out(S_or_B, RM::TMat2PosQuat(M_b_e));
  out.setMeta(
      urdf_model ? urdf_model->name_ : std::string("arm"), // robot_name
      frame_label == "space" ? ScrewList::Rep::Space : ScrewList::Rep::Body, // representation flag
      joint_names,                                    // joint_names
      base_link,                                      // base_frame
      ee_link,                                        // ee_frame
      jl                                              // joint_limits
  );
  return out;
}

// ---------- ROS2 node ----------
class URDFToScrewList : public rclcpp::Node {
public:
  URDFToScrewList() : Node("urdf_to_screw_list") {
    // Parameters
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("ee_link", "tool0");
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("urdf_path", "");         // optional fallback
    this->declare_parameter<std::string>("output_path", "");       // optional file
    this->declare_parameter<std::string>("output_format", "yaml"); // yaml only for now
    this->declare_parameter<bool>("use_body_frame", false);        // false = space S, true = body B
    this->declare_parameter<bool>("verbose", true);
    this->declare_parameter<bool>("home_pose_as_pos_quat", true);

    urdfToScrewList();
  }

private:
  void urdfToScrewList(){
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

    // Build screw_list
    ScrewList screws_ = make_screw_list_from_urdf_string(urdf_xml, base_link, ee_link, use_body);

    // ---- Print to console ----
    const int n = static_cast<int>(screws_.screw_list.cols());

    if(verbose){
      RCLCPP_INFO(get_logger(), "Chain %s -> %s with %d joints.", base_link.c_str(), ee_link.c_str(), n);
      std::string names;
      for(size_t i=0;i<screws_.joint_names.size();++i){
        names += screws_.joint_names[i];
        if(i+1<screws_.joint_names.size()) names += ", ";
      }
      RCLCPP_INFO(get_logger(), "Joint order: [%s]", names.c_str());
    }

    // YAML-ish string (emits M as position + quaternion xyzw)
    std::string out_text;
    {
      std::ostringstream y;
      y.setf(std::ios::fixed); 
      y.precision(10);

      // Robot name
      y << "robot_name: " << screws_.robot_name << "\n";

      // Chain endpoints
      y << "base_link: " << base_link << "\n";
      y << "ee_link: " << ee_link << "\n";

      y << "screw_representation: " << (screws_.screw_representation == ScrewList::Rep::Space ? "space" : "body") << "  # space or body\n";

      // joint_names as list
      y << "joint_names: [";
      for (size_t i = 0; i < screws_.joint_names.size(); ++i) {
        y << screws_.joint_names[i];
        if (i + 1 < screws_.joint_names.size()) y << ", ";
      }
      y << "]\n";

      y << "num_joints: " << n << "\n";

      // screw_list as dictionary mapping joint_name -> [v, w]
      y << "screw_list:\n";
      for (int j = 0; j < n; ++j) {
        VectorXd col = screws_.screw_list.col(j);
        const std::string& jname = screws_.joint_names[j];
        y << "  " << jname << ": [" 
          << col(0) << ", " << col(1) << ", " << col(2) << ", "
          << col(3) << ", " << col(4) << ", " << col(5) << "]\n";
      }

      y << writeJointLimits(screws_);
      y << writeHomePose(screws_, home_pose_as_pos_quat);

      out_text = y.str();
    }

    // Print generate RM format ScrewList
    screws_.PrintList();

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
    auto node = std::make_shared<URDFToScrewList>();  // constructor runs once and finishes
  }  // ensure node is destroyed before shutdown
  rclcpp::shutdown();
  return 0;
}
