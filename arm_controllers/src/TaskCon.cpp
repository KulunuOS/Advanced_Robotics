// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>  //Hardware interface to support commanding an array of joints. 
														//This HardwareInterface supports commanding the output of an array of named joints.

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
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics //Implementation of a recursive forward position kinematics algorithm 
//to calculate the position transformation from joint space to Cartesian space of a general kinematic chain (KDL::Chain). 

// #include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

//
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 97
#define num_taskspace 6
#define A 0.1
#define b 2.5
#define f 1
#define t_set 0.01

namespace arm_controllers
{
class TaskCon : public controller_interface::Controller<hardware_interface::EffortJointInterface> //for commanding effort-based joints. 
{
  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        // ********* 1. Get joint name / gain from the parameter server *********

        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_)) // returns a bool, which provides the ability to check if retrieving the parameter succeeded or not.
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
        Kp_.resize(n_joints_);  //resize the vector to match the number of joints. This will hold P values for PID controller
        Kd_.resize(n_joints_);  //resize the vector to match the number of joints. This will hold D values for PID controller
        Ki_.resize(n_joints_);  //resize the vector to match the number of joints. This will hold I values for PID controller

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_); // create 3 more vectors of double type to fetch and store PID values

        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1); //Sometimes a value must be converted to a literal text form, 
																	  //such as an int represented as a string, or vice-versa, 
																	  //when a string is interpreted as an int, we use this lexical_cast and convert joint number to string value

            if (n.getParam("/elfin/TaskCon/gains/elfin_joint" + si + "/pid/p", Kp[i])) //fetch P values from yaml file.
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/TaskCon/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/TaskCon/gains/elfin_joint" + si + "/pid/i", Ki[i])) //fetch I values from yaml file.
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/TaskCon/gains/elfin_joint" + si + "/pid/d", Kd[i])) //fetch D values from yaml file.
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        // 1.2.2 Closed-loop Inverse Kinematics Controller   //ctr_obj = 1: Regulation; ctr_obj = 2: Tracking
		if (!n.getParam("/elfin/TaskCon/clik_gain/K_regulation", K_regulation_)) //Load Regulation gain
		{
			ROS_ERROR("Cannot find clik regulation gain");
			return false;
		}



        // 2. ********* urdf *********
        urdf::Model urdf;  //creates a class object for Model from urdf. Now, we can access all other child functions from that class!
        if (!urdf.initParam("robot_description")) //Load Model given the name of a parameter on the parameter server. 
													  //return true or false
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
			//ROS_INFO("val of i: %s", i);
			//printf("val of i: %i\n", i);
			//std::cout << "val of i: "<< i << "\n";
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i])); //.push_back Adds a new element at the end of the vector, 
																	   //after its current last element.
																	   //Getting a joint handle through the getHandle() method will claim that resource. 
            }
            catch (const hardware_interface::HardwareInterfaceException &e) //error handler
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);  //creates a shared ptr to each joint
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf); //saves joints locations to joint_urdfs_ by accessing their ptrs... 
        }

        // 4. ********* KDL *********
        // 4.1 kdl parser
		// kdl parser  Constructs a KDL tree from a URDF robot model, first param. is robot model, second kdl_tree_ will 
		// hold the resulting KDL Tree returns true on success

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
        std::string root_name, tip_name;   //string variables to hold the root and tip names (world and elfin_link6)
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
        if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))  //Request the chain of the tree between chain_root and chain_tip.
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints"); //Request the total number of joints in the tree.
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments"); //Request the total number of segments in the tree.
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments(); //fetches different parts of tree. like child , parent etc..
            KDL::SegmentMap::iterator it;  //to iterate over different segments

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // 4.3 inverse dynamics solver 초기화 (initialization)
        gravity_ = KDL::Vector::Zero(); //implementation of a 3 dimensional vector class. a zero vector
        gravity_(2) = -9.81; // 0: x-axis 1: y-axis 2: z-axis //Downwards

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));  //init the id solver by resetting the params to the given specs (kdl_chain_and gravity_)

        // 4.4 jacobian solver 초기화 (initialization)
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));  //init the joint to jacobian solver by resetting the params to the given specs 

        // 4.5 forward kinematics solver 초기화 (initialization)
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));  //init the forward kinematics solver by resetting the params to the given specs 

        // ********* 5. 각종 변수 초기화 (Initialize various variables)*********

        // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0) Initialize (size definition and value 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_); //Dynamic×1 vector of type double with  all coefficients  tau_d is the torque command send to the joints
														 //initialized to zeros same length as number of joints
        x_cmd_.data = Eigen::VectorXd::Zero(num_taskspace); //position command in cartesian/task space

        qd_.data = Eigen::VectorXd::Zero(n_joints_);  //Desired Joint values to be sent to the robot arm.
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_); //Desired Joint angular velocities to be sent to the robot arm.
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_); //Desired Joint acceleration to be sent to the robot arm.
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_); // used to store old desired position value of joint,
        qd_dot_old_.data = Eigen::VectorXd::Zero(n_joints_); // used to store old desired velocity value of joint,

        q_.data = Eigen::VectorXd::Zero(n_joints_);		//Joint positions described by vector q (in rads.)
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);  // Joint angular velocities (in radian/s)

        e_.data = Eigen::VectorXd::Zero(n_joints_);	// store the difference between the desired and current positions
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_); // store the difference between the desired and current velocities
        e_int_.data = Eigen::VectorXd::Zero(n_joints_); // store the integral errors in this vector (initialized to zero) not used???



        // 5.2 KDL Matrix 초기화 (사이즈 정의 및 값 0) (Matrix initialization (size definition and value 0))
        
		J_.resize(kdl_chain_.getNrOfJoints()); //Resize Jacobian matrix
        // J_inv_.resize(kdl_chain_.getNrOfJoints());
        M_.resize(kdl_chain_.getNrOfJoints()); //Resize Inertia matrix
        C_.resize(kdl_chain_.getNrOfJoints()); //Resize coriolis
        G_.resize(kdl_chain_.getNrOfJoints()); //Resize gravity torque vector

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000); //Set publisher to publish the desired joint position values qd (radians)
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000); //Set publisher to publish the current joint position values q (radians)
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000); //Set publisher to publish the joint position Error values e (radians)

        pub_xd_ = n.advertise<std_msgs::Float64MultiArray>("xd", 1000); //Set publisher to publish the desired cartesian/task position values qd_  (meters)
        pub_x_ = n.advertise<std_msgs::Float64MultiArray>("x", 1000); //Set publisher to publish the current cartesian/task position values x_ (meters)
        pub_ex_ = n.advertise<std_msgs::Float64MultiArray>("ex", 1000); //Set publisher to publish the position Error values ex_ (meters)

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // Set publisher to publish all other data under SaveData topic.

        // 6.2 subsriber
        sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &TaskCon::commandCB, this);
        event = 0; // subscribe 받기 전: 0 //initially set to 0. As soon as callback function is called, value is changed to 1, used to send initial set of commands.
                   // subscribe 받은 후: 1

        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg) //Subscriber callback function. Fetch message from buffer and store
    {
        if (msg->data.size() != num_taskspace) // x,y,z and R,P,Y values I guess...
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << num_taskspace << ")! Not executing!");
            return;
        }

        for (int i = 0; i < num_taskspace; i++)
        {
            x_cmd_(i) = msg->data[i];  // x,y,z and R,P,Y Load all
        }

        event = 1; // subscribe 받기 전: 0
                   // subscribe 받은 후: 1
    }

    void starting(const ros::Time &time) //This is called just before the first call to update. 
    {
        t = 0.0;
        ROS_INFO("Starting Computed Torque Controller with Closed-Loop Inverse Kinematics");
    }

    void update(const ros::Time &time, const ros::Duration &period) //This is called periodically when the controller is running. 
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();  //store change in time, every time the update function is triggered.
        t = t + 0.001;

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();  //Joint position values in radians, reads from actual joints current values
            qdot_(i) = joints_[i].getVelocity(); //Angular velocity of Joint in radian/s, reads from actual joints current values
        }

        // 0.3 end-effector state by Compute forward kinematics (x_,xdot_)
        fk_pos_solver_->JntToCart(q_, x_);  //Calculate forward position kinematics for a KDL::Chain, from joint coordinates to cartesian pose.
											//(q_ is input joint values of all joints; x_ are the reference to output cartesian pose) x_ is a KDL frame

        xdot_ = J_.data * qdot_.data; //This tells us that the end-effector velocity is equal to the Jacobian, J multiplied by the joint angle velocity.


        // ********* 1. Desired Trajectory Generation in task space *********
        // *** 1.1 Desired Trajectory in taskspace *** // ctr_obj = 1: Regulation  		// (1) Set Point Regulation

		if (event == 0) // initial command
		{
			xd_.p(0) = 0.5;//0.5;      // x.p: frame position(3x1)
			xd_.p(1) = 0.5;//0.5;
			xd_.p(2) = 0.5;
			xd_.M = KDL::Rotation(KDL::Rotation::RPY(0*D2R, 0*D2R, 0*D2R));  //, x.m: frame orientation (3x3)
		}
		else if (event == 1) // command from ros subscriber  Load commands to desired positions
		{
			xd_.p(0) = x_cmd_(0);
			xd_.p(1) = x_cmd_(1);
			xd_.p(2) = x_cmd_(2);
			xd_.M = KDL::Rotation(KDL::Rotation::RPY(x_cmd_(3), x_cmd_(4), x_cmd_(5))); //KDL::Rotation represents rotations in 3 dimensional space. 
			//Gives back a rotation matrix specified with RPY convention: first rotate around X with roll, then around the old Y with pitch, 
			//then around old Z with yaw
		}

					// Check this place, How to calculate these correctly???
		xd_dot_(0) = 0;  //desired linear velocity
		xd_dot_(1) = 0;
		xd_dot_(2) = 0;
		xd_dot_(3) = 0;
		xd_dot_(4) = 0;
		xd_dot_(5) = 0;

		xd_ddot_(0) = 0;  //desired linear Acceleration
		xd_ddot_(1) = 0;
		xd_ddot_(2) = 0;
		xd_ddot_(3) = 0;
		xd_ddot_(4) = 0;
		xd_ddot_(5) = 0;


        // ********* 2. Inverse Kinematics *********
        // *** 2.0 Error Definition in Task Space ***

        ex_temp_ = diff(x_, xd_);  //use diff command to get the error difference between two frames, here, diff between desired and current frame values.
									//IMETHOD Twist KDL::diff(const Frame &F_a_b1, const Frame &F_a_b2, double dt = 1)
									//determines the rotation axis necessary to rotate the frame b1 to the same orientation as frame b2 
									//and the vector necessary to translate the origin of b1 to the origin of b2, 
									//and stores the result in a Twist datastructure. 
        ex_(0) = ex_temp_(0);
        ex_(1) = ex_temp_(1);
        ex_(2) = ex_temp_(2);
        ex_(3) = ex_temp_(3);
        ex_(4) = ex_temp_(4);
        ex_(5) = ex_temp_(5);

        //ex_dot_ = xd_dot_ - xdot_;  //Difference between desired linear velocity and current linear velocity (which is calculated by jacobian*angular_vel of joints)


        // *** 2.1 computing Jacobian J(q) ***
        jnt_to_jac_solver_->JntToJac(q_, J_); //Calculate the jacobian expressed in the base frame of the chain, with reference point 
											  //at the end effector of the chain. takes joint position array q_ and stores data in J_

        // *** 2.2 computing Jacobian transpose/inversion ***
        J_transpose_ = J_.data.transpose();  //Transpose the Jacobian data and store in variable

        // *** 2.3 computing desired joint state from Open-loop/Closed-loop Inverse Kinematics ***
        // 2.3.1 Regulation Case // ctr_obj = 1: Regulation 
		if (t < t_set)
		{
			qd_.data = qd_old_.data;  //if its not time yet!!....
		}
		else
		{
			qd_.data = qd_old_.data + J_transpose_ * K_regulation_ * ex_ * dt; //Update joint angular positions, old+ Jacobian * Kreg. * task_space_err. * time_element
																				// Refer to Lecture 3 Advanced robotics Slide 20
			qd_old_.data = qd_.data;  //store current desired joint angular positions (radians)
            
            qd_dot_.data = J_transpose_ * K_regulation_ * ex_;
            qd_ddot_.data = (qd_dot_.data - qd_dot_old_.data)/dt;
            qd_dot_old_.data = qd_dot_.data; 
		}



        // ********* 3. Motion Controller in Joint Space*********

        // *** 3.1 Error Definition in Joint Space ***
        e_.data = qd_.data - q_.data;      //difference between desired and current values
        e_dot_.data = qd_dot_.data - qdot_.data;  //difference between desired and current values (velocity)

        e_int_.data = qd_.data - q_.data; // (To do: e_int 업데이트 필요요(need update))

        // *** 3.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);  // calculate inertia matrix H JntToMass(const JntArray &q, JntSpaceInertiaMatrix &H)
        id_solver_->JntToCoriolis(q_, qdot_, C_); //calculate coriolis matrix C, JntToCoriolis(const JntArray &q, const JntArray &q_dot, JntArray &coriolis)
												  //calls CartToJnt(q, q_dot, jntarraynull, wrenchnull, coriolis)

        id_solver_->JntToGravity(q_, G_);   //calculate gravity matrix G (calls CartToJnt internally which is the Function to calculate 
										  //from Cartesian forces to joint torques. "CartToJnt(q, jntarraynull, jntarraynull, wrenchnull, gravity)")

/**ref:  Function to calculate from Cartesian forces to joint torques. Input parameters; 
int KDL::ChainIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches &f_ext, JntArray &torques)

q	The current joint positions
q_dot	The current joint velocities
q_dotdot	The current joint accelerations
f_ext	The external forces (no gravity) on the segments Output parameters:
torques	the resulting torques for the joints 
**/

        // *** 3.3 Apply Torque Command to Actuator ***
        aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data)); // Where is qd_ddot_ calculated???
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data; //refer to lecture 3 PPT slide 27 (full inverse dynamics control)

        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));  //set torque commands to joints
            // joints_[i].setCommand(0.0);
        }

        // ********* 4. data 저장 *********
        //save_data();  //Save all data to publish it over the topic named SaveData

        // ********* 5. state 출력 *********
        print_state(); //prints the data on the console
    }

    void stopping(const ros::Time &time)  //func. to perform when stopping the controller
    {
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

            printf("*** Command from Subscriber in Task Space (unit: m) ***\n");
            if (event == 0)
            {
                printf("No Active!!!\n");
            }
            else
            {
                printf("Active!!!\n");
            }
            printf("x_cmd: %f, ", x_cmd_(0));
            printf("y_cmd: %f, ", x_cmd_(1));
            printf("z_cmd: %f, ", x_cmd_(2));
            printf("r_cmd: %f, ", x_cmd_(3));
            printf("p_cmd: %f, ", x_cmd_(4));
            printf("y_cmd: %f\n", x_cmd_(5));
            printf("\n");

            printf("*** Desired Position in Joint Space (unit: deg) ***\n");
            printf("qd(1): %f, ", qd_(0) * R2D);
            printf("qd(2): %f, ", qd_(1) * R2D);
            printf("qd(3): %f, ", qd_(2) * R2D);
            printf("qd(4): %f, ", qd_(3) * R2D);
            printf("qd(5): %f, ", qd_(4) * R2D);
            printf("qd(6): %f\n", qd_(5) * R2D);
            printf("\n");

            printf("*** Actual Position in Joint Space (unit: deg) ***\n");
            printf("q(1): %f, ", q_(0) * R2D);
            printf("q(2): %f, ", q_(1) * R2D);
            printf("q(3): %f, ", q_(2) * R2D);
            printf("q(4): %f, ", q_(3) * R2D);
            printf("q(5): %f, ", q_(4) * R2D);
            printf("q(6): %f\n", q_(5) * R2D);
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

            // printf("*** Desired Orientation in Task Space (unit: ??) ***\n");
            // printf("xd_(0): %f, ", xd_.M(KDL::Rotation::RPY(0));
            // printf("xd_(1): %f, ", xd_.M(KDL::Rotation::RPY(1));
            // printf("xd_(2): %f\n", xd_.M(KDL::Rotation::RPY(2));
            // printf("\n");

            // printf("*** Actual Orientation in Task Space (unit: ??) ***\n");
            // printf("x_(0): %f, ", x_.M(KDL::Rotation::RPY(0));
            // printf("x_(1): %f, ", x_.M(KDL::Rotation::RPY(1));
            // printf("x_(2): %f\n", x_.M(KDL::Rotation::RPY(2));
            // printf("\n");

            printf("*** Desired Translation Velocity in Task Space (unit: m/s) ***\n");
            printf("xd_dot: %f, ", xd_dot_(0));
            printf("yd_dot: %f, ", xd_dot_(1));
            printf("zd_dot: %f\n", xd_dot_(2));
            printf("\n");

            printf("*** Actual Translation Velocity in Task Space (unit: m/s) ***\n");
            printf("xdot: %f, ", xdot_(0));
            printf("ydot: %f  ", xdot_(1));
            printf("zdot: %f\n", xdot_(2));
            printf("\n");

            printf("*** Desired Angular Velocity in Task Space (unit: rad/s) ***\n");
            printf("rd_dot: %f, ", xd_dot_(3));
            printf("pd_dot: %f, ", xd_dot_(4));
            printf("yd_dot: %f\n", xd_dot_(5));
            printf("\n");

            printf("*** Actual Angular Velocity in Task Space (unit: rad/s) ***\n");
            printf("r_dot: %f, ", xdot_(3));
            printf("p_dot: %f  ", xdot_(4));
            printf("y_dot: %f\n", xdot_(5));
            printf("\n");

            printf("*** Desired Rotation Matrix of end-effector ***\n");
            printf("%f, ",xd_.M(0,0));
            printf("%f, ",xd_.M(0,1));
            printf("%f\n",xd_.M(0,2));
            printf("%f, ",xd_.M(1,0));
            printf("%f, ",xd_.M(1,1));
            printf("%f\n",xd_.M(1,2));
            printf("%f, ",xd_.M(2,0));
            printf("%f, ",xd_.M(2,1));
            printf("%f\n",xd_.M(2,2));
            printf("\n");

            printf("*** Actual Rotation Matrix of end-effector ***\n");
            printf("%f, ",x_.M(0,0));
            printf("%f, ",x_.M(0,1));
            printf("%f\n",x_.M(0,2));
            printf("%f, ",x_.M(1,0));
            printf("%f, ",x_.M(1,1));
            printf("%f\n",x_.M(1,2));
            printf("%f, ",x_.M(2,0));
            printf("%f, ",x_.M(2,1));
            printf("%f\n",x_.M(2,2));
            printf("\n");

            printf("*** Joint Space Error (unit: deg)  ***\n");
            printf("q1: %f, ", R2D * e_(0));
            printf("q2: %f, ", R2D * e_(1));
            printf("q3: %f, ", R2D * e_(2));
            printf("q4: %f, ", R2D * e_(3));
            printf("q5: %f, ", R2D * e_(4));
            printf("q6: %f\n", R2D * e_(5));
            printf("\n");

            printf("*** Task Space Position Error (unit: mm) ***\n");
            printf("x: %f, ", ex_(0)*1000);
            printf("y: %f, ", ex_(1)*1000);
            printf("z: %f\n", ex_(2)*1000);
            printf("\n");

            printf("*** Task Space Orientation Error ?? (unit: deg) ***\n");
            printf("r: %f, ", ex_(3)*R2D);
            printf("p: %f, ", ex_(4)*R2D);
            printf("y: %f\n", ex_(5)*R2D);
            printf("\n");

            printf("*** PID ***\n");
            printf("Kp: %f, ", Kp_(0));
            printf("Kp: %f, ", Kp_(1));
            printf("Kp: %f, ", Kp_(2));
            printf("Kp: %f, ", Kp_(3));
            printf("Kp: %f, ", Kp_(4));
            printf("Kp: %f, ", Kp_(5));

            printf("\n");

            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;
    int event;

    //Joint handles
    unsigned int n_joints_;                               // joint 숫자   //number of joints
    std::vector<std::string> joint_names_;                  //std::vector is a sequence container that encapsulates dynamic size arrays.
    std::vector<hardware_interface::JointHandle> joints_;  //A handle used to read and command a single joint
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;   // joint_urdfs_ is used to refer via shared ptr to a specific joint in robot.

    // kdl
    KDL::Tree kdl_tree_;   // tree
    KDL::Chain kdl_chain_; // chain
		/*** The scoped_ptr class template stores a pointer to a dynamically allocated object. 
			Implementation of a method to calculate the matrices H (inertia),C(coriolis) and G(gravitation) 
			for the calculation torques out of the pose and derivatives. (inverse dynamics)
			It calculates the joint-space inertia matrix, given the motion of the joints (q,qdot,qdotdot), 
			external forces on the segments (expressed in the segments reference frame) and the dynamical parameters of the segments.
		***/


    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector // A fixed size array this will hold gravity values for each joint
    KDL::Vector gravity_;

    // kdl and Eigen Jacobian
    KDL::Jacobian J_;  //creates a container to hold jacobian matrix

    Eigen::Matrix<double, num_taskspace, num_taskspace> J_transpose_;  //a 6x6 matrix

    // kdl solver
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
    // boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;               // Solver To compute the inverse dynamics

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_, qd_dot_old_;  //desired joint angular pos,vel,acc.
    KDL::JntArray qd_old_;  //to store old desired joint angular position
    KDL::JntArray q_, qdot_;  // current joint angular pos, vel
    KDL::JntArray e_, e_dot_, e_int_; // hold error in joint angular position, vel, integral(not used)

    // Task Space State
    // ver. 01
    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_;
    KDL::Twist ex_temp_; //to hold the diff results between x and xd frames

    // KDL::Twist xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> ex_;  //6x1 matrix to hold errors in values of task space
    Eigen::Matrix<double, num_taskspace, 1> xd_dot_, xd_ddot_; //6x1 matrix to hold task space desired velocity (xyz,rpy) and accelerations
    Eigen::Matrix<double, num_taskspace, 1> xdot_; //6x1 matrix to hold task space current velocity (xyz,rpy)
    Eigen::Matrix<double, num_taskspace, 1> ex_dot_, ex_int_; //hold Difference between desired linear velocity and current linear velocity

    // ver. 02
    // Eigen::Matrix<double, num_taskspace, 1> xd_, xd_dot_, xd_ddot_;
    // Eigen::Matrix<double, num_taskspace, 1> x_, xdot_;
    // KDL::Frame x_temp_;
    // Eigen::Matrix<double, num_taskspace, 1> ex_, ex_dot_, ex_int_;

    // Input
    KDL::JntArray x_cmd_; //will hold xyz and rpy values that we want end effector to reach, can get them from subscribed topic.

    // Torque
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;
    double K_regulation_, K_tracking_;

    // save the data
    double SaveData_[SaveDataMax]; // just for the sake of saving data. Maybe in ROS bag

    // ros subscriber
    ros::Subscriber sub_x_cmd_;

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_; //publishers
    ros::Publisher pub_xd_, pub_x_, pub_ex_;
    ros::Publisher pub_SaveData_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;  //message containers
    std_msgs::Float64MultiArray msg_xd_, msg_x_, msg_ex_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::TaskCon, controller_interface::ControllerBase)