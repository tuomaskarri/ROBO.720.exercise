#include "kinematics.hpp"

Kinematics::Kinematics(const float &L1, const float &L2, const float &L3)
    : L1_(L1), L2_(L2), L3_(L3)
{
}

Eigen::Vector2f Kinematics::compute_ee_pos(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Compute end effector position using trigonometrics
     */
    float x = L1_ * std::cos(q1)
            + L2_ * std::cos(q1 + q2)
            + L3_ * std::cos(q1 + q2 + q3);

    float y = L1_ * std::sin(q1)
            + L2_ * std::sin(q1 + q2)
            + L3_ * std::sin(q1 + q2 + q3);

    return Eigen::Vector2f(x, y);
}

Eigen::Matrix3f Kinematics::compute_fk_eigen(const float &q1, const float &q2, const float& q3)
{
    // Construct transformation matrices from each joint to next one
    Eigen::Matrix3f T_base_1, T_1_2, T_2_3, T_3_ee;

    /**
     * TODO: Populate the transformation matrices from joint to joint,
     * then chain them to get the transformation from 'base' to 'end effector'.
     */
    T_base_1 << std::cos(q1), -std::sin(q1), 0.0f,
                std::sin(q1),  std::cos(q1), 0.0f,
                0.0f,         0.0f,          1.0f;

    T_1_2 << std::cos(q2), -std::sin(q2), L1_,
             std::sin(q2),  std::cos(q2), 0.0f,
             0.0f,         0.0f,          1.0f;

    T_2_3 << std::cos(q3), -std::sin(q3), L2_,
             std::sin(q3),  std::cos(q3), 0.0f,
             0.0f,         0.0f,          1.0f;

    T_3_ee << 1.0f, 0.0f, L3_,
              0.0f, 1.0f, 0.0f,
              0.0f, 0.0f, 1.0f;

    Eigen::Matrix3f T_base_ee =
        T_base_1 * T_1_2 * T_2_3 * T_3_ee;

    return T_base_ee;
}

void Kinematics::construct_kdl_chain()
{
    /**
     * TODO: Construct KDL Chain object
     */
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L1_, 0, 0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L2_, 0, 0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(L3_, 0, 0))));
    
}

KDL::Frame Kinematics::compute_fk_kdl(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Construct forward kinematic solver object,
     * and solve end effector pose given the joint values
     */
    KDL::ChainFkSolverPos_recursive solver(chain_);
    KDL::JntArray joint_pos(chain_.getNrOfJoints());
    joint_pos(0) = q1;
    joint_pos(1) = q2;
    joint_pos(2) = q3;

    // Solve
    KDL::Frame result;
    solver.JntToCart(joint_pos, result);
    return result;
}

KDL::JntArray Kinematics::compute_ik_kdl(const float &q1_init, const float &q2_init, const float& q3_init, const KDL::Frame &target_pose)
{
    /**
     * TODO: Construct inverse kinematics solver object,
     * and solve joint positions given the desired pose
     * and initial joint values.
     */
    // Initial joint configuration
    KDL::JntArray initial_joints(chain_.getNrOfJoints());

    initial_joints(0) = q1_init;
    initial_joints(1) = q2_init;
    initial_joints(2) = q3_init;

    // inverse kinematic solver
    KDL::ChainIkSolverPos_LMA ik_solver(chain_);

    // Resulting joint configuration
    KDL::JntArray result_joints(chain_.getNrOfJoints());

    int status = ik_solver.CartToJnt(
        initial_joints,
        target_pose,
        result_joints
    );

    return result_joints;
}

KDL::Jacobian Kinematics::compute_jac_kdl(const float &q1, const float &q2, const float& q3,
                                          const int& segment_n)
{
    /**
     * TODO: Construct Jacobian solver object, and
     * solve the Jacobian given the joint values.
     */

     KDL::JntArray joint_pos(chain_.getNrOfJoints());

     joint_pos(0) = q1;
     joint_pos(1) = q2;
     joint_pos(2) = q3;


     // Jacobian solver
     KDL::ChainJntToJacSolver jac_solver(chain_);

     KDL::Jacobian jacobian(chain_.getNrOfJoints());

     int status = jac_solver.JntToJac(
        joint_pos,
        jacobian,
        segment_n
     );

    return jacobian;
}
