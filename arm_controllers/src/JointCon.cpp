// from ros-control meta packages
#include <controller_interface/controller.h> 
#include <hardware_interface/joint_command_interface.h> //Hardware interface to support commanding an array of joints. 
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

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 43

namespace arm_controllers
{
class JointCon : public controller_interface::Controller<hardware_interface::EffortJointInterface> //for commanding effort-based joints. 
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
		//ROS_INFO("MY Dbug Got param: %s", joint_names_);
        n_joints_ = joint_names_.size();
		std::cout << "n_joints: " << n_joints_ << "\n";
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
        Kp1_.resize(n_joints_);
        Kp2_.resize(n_joints_);
        Kp3_.resize(n_joints_);
        Kd_.resize(n_joints_);  //resize the vector to match the number of joints. This will hold D values for PID controller
        Kd1_.resize(n_joints_);
        Kd2_.resize(n_joints_);
        Kd3_.resize(n_joints_);

        std::vector<double> Kp(n_joints_), Kd(n_joints_); // create 2 more vectors of double type to fetch and store PD values
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1); //Sometimes a value must be converted to a literal text form, 
																	  //such as an int represented as a string, or vice-versa, 
																	  //when a string is interpreted as an int, we use this lexical_cast and convert joint number to string value

            if (n.getParam("/elfin/JointCon/gains/elfin_joint" + si + "/pid1/p", Kp[i])) //fetch P values from yaml file.
            {
//                Kp_(i) = Kp[i]; 
                Kp1_(i) = Kp[i]; 
                n.getParam("/elfin/JointCon/gains/elfin_joint" + si + "/pid2/p", Kp[i]);
                Kp2_(i) = Kp[i]; 
                n.getParam("/elfin/JointCon/gains/elfin_joint" + si + "/pid3/p", Kp[i]);
                Kp3_(i) = Kp[i]; 
            }
            else
            {
                //std::cout << "/elfin/JointCon/gains/elfin_joint" + si + "/pid" + mode_str + "/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/JointCon/gains/elfin_joint" + si + "/pid1/d", Kd[i])) //fetch D values from yaml file.
            {
                Kd1_(i) = Kd[i];
//                Kd_(i) = Kd[i];
                n.getParam("/elfin/JointCon/gains/elfin_joint" + si + "/pid2/d", Kd[i]);
                Kd2_(i) = Kd[i];
                n.getParam("/elfin/JointCon/gains/elfin_joint" + si + "/pid3/d", Kd[i]);
                Kd3_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
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
            joint_urdfs_.push_back(joint_urdf); //saves joints locations to joint_urdfs_ by accessing their ptrs... Not used anywhere???
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
		//std::cout << "root_link: " << root_name << "\n";  //world
		//std::cout << "tip_link: " << tip_name << "\n";   //elfin_link6

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
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis //Downwards

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));  //init the id solver by resetting the params to the given specs (kdl_chain_and gravity_)

        // ********* 5. 각종 변수 초기화 (Initialize various variables) *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0) (Vector initialization (size definition and value 0))
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_); //Dynamic×1 vector of type double with  all coefficients  tau_d is the torque command send to the joints
														 //initialized to zeros same length as number of joints
        q_cmd_.data = Eigen::VectorXd::Zero(n_joints_); //Receive Joint angles command to control robot in joint space
        qd_.data = Eigen::VectorXd::Zero(n_joints_);  //Desired Joint values to be sent to the robot arm.
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_); //Desired Joint angular velocities to be sent to the robot arm.
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_); //Desired Joint acceleration to be sent to the robot arm.
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_); // not used here.... can be used to store old cmd value of joint,

        q_.data = Eigen::VectorXd::Zero(n_joints_);		//Joint positions described by vector q (in rads.)
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);  // Joint angular velocities (in radian/s)

        e_.data = Eigen::VectorXd::Zero(n_joints_);	// store the difference between the desired and current positions
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_); // store the difference between the desired and current velocities

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0) (Matrix initialization (size definition and value 0))
        M_.resize(kdl_chain_.getNrOfJoints()); //Resize Inertia matrix
        C_.resize(kdl_chain_.getNrOfJoints()); //Resize coriolis
        G_.resize(kdl_chain_.getNrOfJoints()); //Resize gravity torque vector

        // ********* 6. ROS 명령어 (ROS command) *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000); //Set publisher to publish the desired joint position values qd (radians)
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000); //Set publisher to publish the current joint position values q (radians)
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000); //Set publisher to publish the joint position Error values e (radians)

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // Set publisher to publish all other data under SaveData topic.
		//maybe we can calculate the cartesian values and publish those too.

        // 6.2 subsriber
        sub_q_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointCon::commandCB, this);
        event = 0; // subscribe 받기 전: 0 //initially set to 0. As soon as callback function is called, value is changed to 1, used to send initial set of commands.
                   // subscribe 받은 후: 1
        mode = 1;  //to determine which PID values to load.
        
        if (mode == 1)
        {
            Kp_.data = Kp1_.data;
            Kd_.data = Kd1_.data;
        }
        else if (mode==2)
        {
            Kp_.data = Kp2_.data;
            Kd_.data = Kd2_.data;
        }
        else if (mode==3)
        {
            Kp_.data = Kp3_.data;
            Kd_.data = Kd3_.data;
        }
        //std::cout << Kp_.data << Kd_.data;
        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg) //Subscriber callback function. Fetch message from buffer and store
    {
        if (msg->data.size() != n_joints_+1) // x,y,z and R,P,Y values I guess...
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints on the robot (" << n_joints_+1 << ")! Not executing!");
            return;
        }

        for (int i = 0; i < n_joints_; i++)
        {
            q_cmd_(i) = msg->data[i];  // x,y,z and R,P,Y Load all
        }
        mode = msg->data[6]; //tag which tells the mode
        event = 1; // subscribe 받기 전: 0
                   // subscribe 받은 후: 1
    }


    void starting(const ros::Time &time) //This is called just before the first call to update. 
    {
        t = 0.0;
        ROS_INFO("Starting Computed Torque Controller");
    }

    void update(const ros::Time &time, const ros::Duration &period) //This is called periodically when the controller is running. 
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();  //store change in time, every time the update function is triggered.
        t = t + 0.001;

        if (mode == 1)
        {
            Kp_.data = Kp1_.data;
            Kd_.data = Kd1_.data;
        }
        else if (mode==2)
        {
            Kp_.data = Kp2_.data;
            Kd_.data = Kd2_.data;
        }
        else if (mode==3)
        {
            Kp_.data = Kp3_.data;
            Kd_.data = Kd3_.data;
        }

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();  //Joint position values in radians, reads from actual joints current values
            qdot_(i) = joints_[i].getVelocity(); //Angular velocity of Joint in radian/s, reads from actual joints current values
        }

        // ********* 1. Desired Trajectory in Joint Space *********
		if (event == 0) // initial command
		{
            qd_(0) = D2R*4;  //0.08620276405114825
            qd_(1) = D2R*-13;  //-0.23630689836981292
            qd_(2) = D2R*75;  //1.3120695430934290471
            qd_(3) = D2R*3;  //0.05160581880770909
            qd_(4) = D2R*90;  //1.5771275790224242
            qd_(5) = D2R*180;  //3.139946935438227
		}
		else if (event == 1) // command from ros subscriber  Load commands to desired joint angles
		{
            qd_(0) = q_cmd_(0);
            qd_(1) = q_cmd_(1);
            qd_(2) = q_cmd_(2);
            qd_(3) = q_cmd_(3);
            qd_(4) = q_cmd_(4);
            qd_(5) = q_cmd_(5);
		}

        for (size_t i = 0; i < n_joints_; i++)
        {
            qd_ddot_(i) = 0.0;//-M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2 * t);  //M_PI is Pi value defined in math library c++ (very long value) qd_ddot_ is holding the acceleration values
            qd_dot_(i) = 0.0;//M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2 * t);          // store velocity values
		}

        // ********* 2. Motion Controller in Joint Space*********
        // *** 2.1 Error Definition in Joint Space ***
		//No angle error handler found!!!

        e_.data = qd_.data - q_.data;      //difference between desired and current values
        e_dot_.data = qd_dot_.data - qdot_.data;  //difference between desired and current values (velocity)

        // *** 2.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);  // calculate inertia matrix H JntToMass(const JntArray &q, JntSpaceInertiaMatrix &H)


        id_solver_->JntToCoriolis(q_, qdot_, C_); //calculate coriolis matrix C, JntToCoriolis(const JntArray &q, const JntArray &q_dot, JntArray &coriolis)
												  //calls CartToJnt(q, q_dot, jntarraynull, wrenchnull, coriolis)


        id_solver_->JntToGravity(q_, G_);  //calculate gravity matrix G (calls CartToJnt internally which is the Function to calculate 
										  //from Cartesian forces to joint torques. "CartToJnt(q, jntarraynull, jntarraynull, wrenchnull, gravity)")

/**ref:  Function to calculate from Cartesian forces to joint torques. Input parameters; 
int KDL::ChainIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches &f_ext, JntArray &torques)

q	The current joint positions
q_dot	The current joint velocities
q_dotdot	The current joint accelerations
f_ext	The external forces (no gravity) on the segments Output parameters:
torques	the resulting torques for the joints 
**/


        // *** 2.3 Apply Torque Command to Actuator ***
		
		//std::cout << "Kp: "<< Kp_.data << "\n";
        aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
        //aux_d_.data = M_.data * (qd_ddot_.data + Kd_.data.cwiseProduct(e_dot_.data));
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data; //refer to lecture 3 PPT slide 27 (full inverse dynamics control)

        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));  //set torque commands to joints
            // joints_[i].setCommand(0.0);
        }

        // ********* 3. data 저장 *********
        save_data();  //Save all data to publish it over the topic named SaveData

        // ********* 4. state 출력 *********
        print_state(); //prints the data on the console
    }

    void stopping(const ros::Time &time)  //func. to perform when stopping the controller
    {
    }

    void save_data() //Save all data to publish it over the topic named SaveData
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Desired position in joint space (unit: rad)
        SaveData_[1] = qd_(0);
        SaveData_[2] = qd_(1);
        SaveData_[3] = qd_(2);
        SaveData_[4] = qd_(3);
        SaveData_[5] = qd_(4);
        SaveData_[6] = qd_(5);

        // Desired velocity in joint space (unit: rad/s)
        SaveData_[7] = qd_dot_(0);
        SaveData_[8] = qd_dot_(1);
        SaveData_[9] = qd_dot_(2);
        SaveData_[10] = qd_dot_(3);
        SaveData_[11] = qd_dot_(4);
        SaveData_[12] = qd_dot_(5);

        // Desired acceleration in joint space (unit: rad/s^2)
        SaveData_[13] = qd_ddot_(0);
        SaveData_[14] = qd_ddot_(1);
        SaveData_[15] = qd_ddot_(2);
        SaveData_[16] = qd_ddot_(3);
        SaveData_[17] = qd_ddot_(4);
        SaveData_[18] = qd_ddot_(5);

        // Actual position in joint space (unit: rad)
        SaveData_[19] = q_(0);
        SaveData_[20] = q_(1);
        SaveData_[21] = q_(2);
        SaveData_[22] = q_(3);
        SaveData_[23] = q_(4);
        SaveData_[24] = q_(5);

        // Actual velocity in joint space (unit: rad/s)
        SaveData_[25] = qdot_(0);
        SaveData_[26] = qdot_(1);
        SaveData_[27] = qdot_(2);
        SaveData_[28] = qdot_(3);
        SaveData_[29] = qdot_(4);
        SaveData_[30] = qdot_(5);

        // Error position in joint space (unit: rad)
        SaveData_[31] = e_(0);
        SaveData_[32] = e_(1);
        SaveData_[33] = e_(2);
        SaveData_[34] = e_(3);
        SaveData_[35] = e_(4);
        SaveData_[36] = e_(5);

        // Error velocity in joint space (unit: rad/s)
        SaveData_[37] = e_dot_(0);
        SaveData_[38] = e_dot_(1);
        SaveData_[39] = e_dot_(2);
        SaveData_[40] = e_dot_(3);
        SaveData_[41] = e_dot_(4);
        SaveData_[42] = e_dot_(5);

        // 2
        msg_qd_.data.clear();  //clean data buffers before loading and publishing to their respective topics
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)  //Load data to their msg buffers
        {
            msg_qd_.data.push_back(qd_(i)); //desired joint position
            msg_q_.data.push_back(q_(i)); //current joint position
            msg_e_.data.push_back(e_(i)); //difference (error) joint position
        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]); //all of the data stuffed in savedata buffer
        }

        // 4
        pub_qd_.publish(msg_qd_);  //publish
        pub_q_.publish(msg_q_);  //publish
        pub_e_.publish(msg_e_);  //publish

        pub_SaveData_.publish(msg_SaveData_);  //publish
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
                printf("No Active commands!!!\n");
            }
            else
            {
                printf("Active commands!!!\n");
            }
            printf("*** Desired State in Joint Space (unit: deg) ***\n");
            printf("qd_(0): %f, ", qd_(0)*R2D);
            printf("qd_(1): %f, ", qd_(1)*R2D);
            printf("qd_(2): %f, ", qd_(2)*R2D);
            printf("qd_(3): %f, ", qd_(3)*R2D);
            printf("qd_(4): %f, ", qd_(4)*R2D);
            printf("qd_(5): %f\n", qd_(5)*R2D);
            printf("\n");

            printf("*** Actual State in Joint Space (unit: deg) ***\n");
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            printf("\n");


            printf("*** Joint Space Error (unit: deg)  ***\n");
            printf("%f, ", R2D * e_(0));
            printf("%f, ", R2D * e_(1));
            printf("%f, ", R2D * e_(2));
            printf("%f, ", R2D * e_(3));
            printf("%f, ", R2D * e_(4));
            printf("%f\n", R2D * e_(5));
            printf("\n");

            printf("*** PID ***\n");
            printf("Kp: %f, ", Kp_(0));
            printf("Kp: %f, ", Kp_(1));
            printf("Kp: %f, ", Kp_(2));
            printf("Kp: %f, ", Kp_(3));
            printf("Kp: %f, ", Kp_(4));
            printf("Kp: %f ", Kp_(5));
            printf("\n");

            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;
    int event;
    int mode;
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

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;
    KDL::JntArray q_cmd_; //will hold joint values for all joints for our robot to reach, can get them from subscribed topic.

    // gains
    KDL::JntArray Kp_,Kp1_,Kp2_,Kp3_, Kd_, Kd1_, Kd2_, Kd3_;

    // save the data
    double SaveData_[SaveDataMax]; // just for the sake of saving data. Maybe in ROS bag

    // ros subscriber
    ros::Subscriber sub_q_cmd_;

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::JointCon, controller_interface::ControllerBase)