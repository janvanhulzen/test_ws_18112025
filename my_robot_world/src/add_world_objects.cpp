#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <geometric_shapes/shape_operations.h>   // createMeshFromResource, constructMsgFromShape
#include <Eigen/Geometry>                        // Eigen::Vector3d
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//obj
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_world_objects");

  moveit::planning_interface::PlanningSceneInterface psi;

  // 1) Load and SCALE the STL (mm -> m)
  // const std::string mesh_uri = "package://my_robot_world/meshes/game.stl";
  // const Eigen::Vector3d scale(0.0001, 0.0001, 0.0001);  // 1 mm = 0.001 m
  // shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_uri, scale);

  // 1) Load and SCALE the obj (mm -> m)
  const std::string mesh_uri = "package://my_robot_world/meshes/game.obj";
  const Eigen::Vector3d scale(0.0001, 0.0001, 0.0001);   // if OBJ is in mm
  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_uri, scale);

// convert to shape_msgs::Mesh and add to CollisionObject as before…


  if (!mesh)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load mesh: %s", mesh_uri.c_str());
    rclcpp::shutdown();
    return 1;
  }

  shape_msgs::msg::Mesh mesh_msg;
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);
  mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  // 2) Collision object setup
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "base_link";  // your planning/world frame
  obj.id = "game";
  obj.operation = obj.ADD;

  geometry_msgs::msg::Pose pose;
  
  tf2::Quaternion q;
  q.setRPY( M_PI/2.0,0.0,M_PI/2.0);   // roll=0, pitch=0, yaw=+90° around Z
  pose.orientation = tf2::toMsg(q);
  pose.position.x = 0.3;   // meters
  pose.position.y = 0.0;
  pose.position.z = 0.0;

  obj.meshes.push_back(mesh_msg);
  obj.mesh_poses.push_back(pose);

  // 3) Apply to planning scene
  psi.applyCollisionObjects({obj});

  // Free heap mesh (constructMsgFromShape copies the data)
  delete mesh;

  // Give it a moment to publish
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  rclcpp::shutdown();
  return 0;
}
