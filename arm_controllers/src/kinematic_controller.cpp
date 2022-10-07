// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define num_taskspace 6
#define A 0.1
#define b 2.5
#define f 1
#define t_set 1



namespace arm_controllers
{
class KinematicController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
  public:
    bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n)
    {
        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }

        // 1.2 Gain
        // 1.2.1 Joint Controller
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/kinematic_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/kinematic_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/kinematic_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/kinematic_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        // 2. ********* urdf *********
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        else
        {
            ROS_INFO("Found robot_description");
        }

        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }

        // 4. ********* KDL *********
        // 4.1 kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

        // 4.2 kdl chain
        std::string root_name, tip_name;
        if (!n.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_name))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }
        if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // Switch between jointspace and taskspace
        if (!n.getParam("space_mode", space_mode_))
        {
            ROS_ERROR("Could not find space mode (joint / space) ? ");
            return false;
        }

        else
        {
            ROS_INFO("Starting control in '%s' space", space_mode_.c_str());
        }

        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

        // 4.4 jacobian solver 초기화
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

        // 4.5 forward kinematics solver 초기화
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);
        //q_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        
        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
            // Joint space
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);
        pub_qd_dot_ = n.advertise<std_msgs::Float64MultiArray>("qd_dot", 1000);
        pub_q_dot_ = n.advertise<std_msgs::Float64MultiArray>("q_dot", 1000);
            // Task space
        //pub_xd_ = n.advertise<std_msgs::Float64MultiArray>("xd", 1000);
        //pub_x_ = n.advertise<std_msgs::Float64MultiArray>("x", 1000);
        //pub_ex_ = n.advertise<std_msgs::Float64MultiArray>("ex", 1000);

        //pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber

        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != n_joints_)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Kinematic Controller");
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();
        t = t + 0.001;

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }


        // ********* 2. Motion Controller in Joint Space*********
        if (space_mode_ == "joint")
        {
            // ********* 1. Desired Trajecoty in Joint Space (qd_ and qd_dot_) *********
            for (size_t i = 0; i < n_joints_; i++)
            {
                //qd_ddot_(i) = -M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2 * t); 
                qd_dot_(i) = M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2 * t);          
                qd_(i) = 45 * KDL::deg2rad * sin(M_PI / 2* t);
            }
            // *** 2.1 Error Definition in Joint Space ***
            e_.data = qd_.data - q_.data;
            e_dot_.data = qd_dot_.data - qdot_.data;
            e_int_.data = qd_.data - q_.data; // (To do: e_int 업데이트 필요요)

            // *** 2.2 Compute model(M,C,G) ***
            // id_solver_->JntToMass(q_, M_);
            // id_solver_->JntToCoriolis(q_, qdot_, C_);
            // id_solver_->JntToGravity(q_, G_); 

            // *** 2.3 Apply Torque Command to Actuator ***
            // change the control command equation here <<

            //aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
            //comp_d_.data = C_.data + G_.data;
            //tau_d_.data = aux_d_.data + comp_d_.data;
            q_cmd_.data = qd_dot_.data + Kp_.data.cwiseProduct(e_.data);


            // set the command
            for (int i = 0; i < n_joints_; i++)
            {
                 joints_[i].setCommand(q_cmd_(i));
                 // joints_[i].setCommand(0.0);
            }


        }

        if (space_mode_ == "task")
        {
            // ********* 1. Desired Trajecoty in Task Space (xd_ and xd_dot_) *********
            xd_.p(0) = 0.1;
            xd_.p(1) = A * sin(f * M_PI * (t - t_set)) + b;
            xd_.p(2) = 0.5;
            xd_.M = KDL::Rotation(KDL::Rotation::RPY(0, 0, 0));

            xd_dot_(0) = 0;
            xd_dot_(1) = (f * M_PI) * A * cos(f * M_PI * (t - t_set));
            xd_dot_(3) = 0;
            xd_dot_(4) = 0;
            xd_dot_(5) = 0;

            xd_ddot_(0) = 0;
            xd_ddot_(1) = -1 * (f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - t_set));
            xd_ddot_(2) = 0;
            xd_ddot_(3) = 0;
            xd_ddot_(4) = 0;
            xd_ddot_(5) = 0;
            
            
            // Calculate end effector state by computing forward kienmatics x_ / ( xd_dot is optional)
            fk_pos_solver_->JntToCart(q_, x_);
            ex_temp_ = diff(x_, xd_);

            Xerr_(0) = ex_temp_(0);
            Xerr_(1) = ex_temp_(1);
            Xerr_(2) = ex_temp_(2);
            Xerr_(3) = ex_temp_(3);
            Xerr_(4) = ex_temp_(4);
            Xerr_(5) = ex_temp_(5);

            ex_(0) = x_.p(0);
            ex_(1) = x_.p(1);
            ex_(2) = x_.p(2);
            x_.M.GetRPY(ex_(3),ex_(4), ex_(5));

            // ex_(3) = x_.M(0);
            // ex_(4) = x_.M(1);
            // ex_(5) = x_.M(2);
        
            exd_(0) = xd_.p(0);
            exd_(1) = xd_.p(1);
            exd_(2) = xd_.p(2);
            xd_.M.GetRPY(exd_(3), exd_(4), exd_(5));
          //  exd_(3) = xd_.M(0);
          //  exd_(4) = xd_.M(1);
          //  exd_(5) = xd_.M(2);
            

            jnt_to_jac_solver_->JntToJac(q_, J_);
            Vcmd = xd_dot_ + Kp_.data.cwiseProduct(Xerr_) ;
            J_inv_ = J_.data.inverse();
            // pseudo_inverse(J_.data,J_inv_.data,false);
            q_dot_cmd_ = J_inv_.cwiseProduct(Vcmd);


            q_cmd_(0) = q_dot_cmd_(0);
            q_cmd_(1) = q_dot_cmd_(1);
            q_cmd_(2) = q_dot_cmd_(2);
            q_cmd_(3) = q_dot_cmd_(3);
            q_cmd_(4) = q_dot_cmd_(4);
            q_cmd_(5) = q_dot_cmd_(5);
            // set the command
            for (int i = 0; i < n_joints_; i++)
            {
                 joints_[i].setCommand(q_cmd_(i));
                 // joints_[i].setCommand(0.0);
            }            

        }

        // ********* 3. data 저장 *********
        // save_data();

        // ********* 4. state 출력 *********
        //print_state();
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
 
        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();
        msg_qd_dot_.data.clear();
        msg_q_dot_.data.clear();

        msg_x_.data.clear();
        msg_xd_.data.clear();
        msg_ex_.data.clear();

        //msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
            msg_qd_dot_.data.push_back(qd_dot_(i));
            msg_q_dot_.data.push_back(qdot_(i));

            
        }

        for (int i = 0; i < num_taskspace; i++)
         {
             msg_xd_.data.push_back(exd_(i));
             msg_x_.data.push_back(ex_(i));
             msg_ex_.data.push_back(Xerr_(i));
         }      
        //for (int i = 0; i < SaveDataMax; i++)
        //{
        //    msg_SaveData_.data.push_back(SaveData_[i]);
        //}

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);
        pub_qd_dot_.publish( msg_qd_dot_);
        pub_q_dot_.publish( msg_q_dot_);

        pub_x_.publish(msg_x_);
        pub_xd_.publish(msg_xd_);
        pub_ex_.publish(msg_ex_);
        //pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 99)
        {
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            printf("*** Desired Position in Task Space (unit: m) ***\n");
            printf("xd: %f, ", xd_.p(0));
            printf("yd: %f, ", xd_.p(1));
            printf("zd: %f\n", xd_.p(2));
            printf("\n");

            printf("*** Actual Position in Task Space (unit: m) ***\n");
            printf("x: %f, ", x_.p(0));
            printf("y: %f, ", x_.p(1));
            printf("z: %f\n", x_.p(2));
            printf("\n");

            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;
    std::string space_mode_;
    //Joint handles
    unsigned int n_joints_;                               // joint 숫자
    std::vector<std::string> joint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> joints_; // ??
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

    // kdl
    KDL::Tree kdl_tree_;   // tree?
    KDL::Chain kdl_chain_; // chain?

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    // kdl and Eigen Jacobian
    KDL::Jacobian J_;
    Eigen::MatrixXd J_inv_;
    Eigen::Matrix<double, num_taskspace, num_taskspace> J_transpose_;

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_, e_int_;

    // Task Space State
    // ver. 01
    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_;
    KDL::Twist ex_temp_;
    Eigen::Matrix<double, num_taskspace, 1> ex_ , exd_;

    // KDL::Twist xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> Xerr_;
    Eigen::Matrix<double, num_taskspace, 1> xd_dot_, xd_ddot_ , Vcmd, q_dot_cmd_;
    Eigen::Matrix<double, num_taskspace, 1> xdot_;
    Eigen::Matrix<double, num_taskspace, 1> ex_dot_, ex_int_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_ , q_cmd_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_ ,pub_qd_dot_, pub_q_dot_;
    ros::Publisher pub_xd_, pub_x_, pub_ex_;
    ros::Publisher pub_SaveData_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_, msg_qd_dot_,msg_q_dot_;
    std_msgs::Float64MultiArray msg_SaveData_;
    std_msgs::Float64MultiArray msg_x_, msg_xd_, msg_ex_;

};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::KinematicController, controller_interface::ControllerBase)