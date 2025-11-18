#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    
    // we need a new thread at this point to spin the node
    // - thread with instructions to move the robot
    // - thread to spin the node

    // create single threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // create rest of code here

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    arm.setStartStateToCurrentState();

    if (false){
        tf2::Quaternion q;
        q.setRPY(0.0,0.0,3.14/6.0); // roll, pitch, yaw
        q=q.normalize();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x =  0.310862;
        target_pose.pose.position.y =  0.0;//-0.179703;
        target_pose.pose.position.z =  0.310862;
        target_pose.pose.orientation.x = q.x(); //0.707388;
        target_pose.pose.orientation.y = q.y(); //0.706825;
        target_pose.pose.orientation.z = q.z(); //-0.000951175;
        target_pose.pose.orientation.w = q.w(); //-0.000473876;

        arm.setPoseTarget(target_pose);
        // plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        // execute the motion plan
        bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success1) {
            arm.execute(plan1);    // execute the motion plan 1
        }
    }


    if (true){
        arm.setNamedTarget("pose_1"); // from srdf
        // plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan1,plan2,plan3;
        
        // execute the motion plan
        bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success1) {
            arm.execute(plan1);    // execute the motion plan 1
            arm.setStartStateToCurrentState();
            arm.setNamedTarget("pose_2");
            // execute the motion plan
            bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success2) {
                arm.execute(plan2);    // execute the motion plan 2
                arm.setStartStateToCurrentState();
                arm.setNamedTarget("home"); // from srdf
                bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success3) {
                    arm.execute(plan3);     // execute the motion plan 3
                }
            }
        }
    }


    rclcpp::shutdown();
    spinner.join();
    return 0;
}