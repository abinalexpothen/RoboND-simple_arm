#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

class LookAwayPublishSubscribe
{
public:
  LookAwayPublishSubscribe()
  {
    ROS_INFO_STREAM("Init LookAwayPublishSubscribe constructor");

    // Define joint publishers
    joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    //Define subscribers
    joints_sub = n.subscribe("/simple_arm/joint_states", 10, &LookAwayPublishSubscribe::move_state_callback, this);
    camera_sub = n.subscribe("rgb_camera/image_raw", 10, &LookAwayPublishSubscribe::look_away_callback, this);
    
    last_position.push_back(0);
    last_position.push_back(0);

    moving_state = false;
  }

  // callback function that checks if robot is moving
  void move_state_callback(const sensor_msgs::JointState joint_state)
  {
      std::vector<double> current_position = joint_state.position;
  
      double tolerance = 0.0001;
  
      // check if arm is moving
      if (fabs(current_position[0] - last_position[0]) < tolerance && fabs(current_position[1] - last_position[1]) < tolerance)
          moving_state = false;
      else {
          moving_state = true;
          last_position = current_position;
      }
  }
  
  // callback function that reads image and publishes to the joint 
  void look_away_callback(const sensor_msgs::Image img)
  {
      bool uniform_image = true;
  
      // check for image uniformity
      for (int i = 0; i < img.height * img.step; i++) {
          if (img.data[i] - img.data[0] != 0) {
              uniform_image = false;
              break;
          }
      }
  
      // point arm to the blocks
      if (uniform_image == true && moving_state == false)
      {
        ROS_INFO_STREAM("Pointing arm to the blocks.");
        std_msgs::Float64 joint1_val, joint2_val;
        joint1_val.data = 1.57;
        joint2_val.data = 1.57;
        joint1_pub.publish(joint1_val);
        joint2_pub.publish(joint2_val);

        // Wait 3 seconds for arm to settle
        ros::Duration(4).sleep(); 
      }
  }

private:
  ros::NodeHandle n; 
  ros::Publisher joint1_pub, joint2_pub;
  ros::Subscriber joints_sub, camera_sub;
  
  std::vector<double> last_position;
  bool moving_state;

};//End of class LookAwayPublishSubscribe

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "look_away_publish_subscribe");

  LookAwayPublishSubscribe LAPSObject;

  ros::spin();

  return 0;
}