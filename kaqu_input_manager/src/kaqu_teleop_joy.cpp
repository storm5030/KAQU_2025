// joy 메세지를 받고 소프 4팀이 정의한 커스텀 메시지로 변환시켜주는 노드
// 어떤 알고리즘(계산방식)으로 변환할지 고민 필요

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "std_msgs/msg/string.hpp"
#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "kaqu_msgs/msg/joy_ctrl_cmds.hpp"

#include "kaqu_definitions.hpp"

using std::placeholders::_1;

int t1, t2, t3 = clock();
int btn_tgl_delay = 3000;

auto cmd = kaqu_msgs::msg::JoyCtrlCmds();

bool LJ_btn_sw = 0;

class KaQuGamepadNode : public rclcpp::Node
{
  public: 
    KaQuGamepadNode() 
    : Node("kaqu_gamepad_node")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 12, std::bind(&KaQuGamepadNode::topic_callback, this, _1));

      publisher_ = this->create_publisher<kaqu_msgs::msg::JoyCtrlCmds>("kaqu_joy_ctrl_cmd",40);
    }

    void joy_state_to_joy_cmd(sensor_msgs::msg::Joy::SharedPtr msg_joy)
    { 
      // set start <- btn: start
      if (!cmd.states[0] && msg_joy->buttons[7] && (clock() - t1 > btn_tgl_delay)){
        cmd.states[0] = true;
        t1 = clock();
        RCLCPP_INFO(this->get_logger(), "Start button pressed: ON");
        // button[7]: start button
      }
      else if (cmd.states[0] && msg_joy->buttons[7] && (clock() - t1 > btn_tgl_delay )){
        cmd.states[0] = false;
        t1 = clock();
        RCLCPP_INFO(this->get_logger(), "Start button pressed: OFF");
      }

      if (cmd.states[0]){
        // set walk <- btn: back
        if (!cmd.states[1] && msg_joy->buttons[6] && (clock() - t2 > btn_tgl_delay)){
          cmd.states[1] = true;
          t2 = clock();
          RCLCPP_INFO(this->get_logger(), "Walk mode: ON");
          // button[6]: 뒤로 버튼(우리 조이스틱: select button)
        }
        else if (cmd.states[1] && msg_joy->buttons[6] && (clock() - t2 > btn_tgl_delay )){
          cmd.states[1] = false;
          t2 = clock();
          RCLCPP_INFO(this->get_logger(), "Walk mode: OFF");
        }

        // change side_walk_mode <- btn: RJ btn
        if (!cmd.states[2] && msg_joy->buttons[10] && (clock() - t3 > btn_tgl_delay)){
          cmd.states[2] = true;
          t3 = clock();
          RCLCPP_INFO(this->get_logger(), "Side walk mode: ON");
          // button[10]: 오른쪽 스틱 누르기
        }
        else if (cmd.states[2] && msg_joy->buttons[10] && (clock() - t3 > btn_tgl_delay )){
          cmd.states[2] = false;
          t3 = clock();
          RCLCPP_INFO(this->get_logger(), "Side walk mode: OFF");
        }

        // select gait <- btn: A, B, X, Y
        if(msg_joy->buttons[0]){
          cmd.gait_type = 0;
          RCLCPP_INFO(this->get_logger(), "Gait type: A");
          // button[0]: A
        }
        if(msg_joy->buttons[1]){
          cmd.gait_type  = 1;
          RCLCPP_INFO(this->get_logger(), "Gait type: B");
          // button[1]: B
        }
        if(msg_joy->buttons[2]){
          cmd.gait_type  = 2;
          RCLCPP_INFO(this->get_logger(), "Gait type: X");
          // button[2]: X
        }
        if(msg_joy->buttons[3]){
          cmd.gait_type = 3;
          RCLCPP_INFO(this->get_logger(), "Gait type: Y");
          // button[3]: Y
        }

        // set robot height
        if (cmd.pose.position.z < MIN_HEIGHT)
          cmd.pose.position.z = MIN_HEIGHT;
        if(cmd.pose.position.z > MAX_HEIGHT)
          cmd.pose.position.z = MAX_HEIGHT;

        if(!msg_joy->buttons[4] && msg_joy->axes[7] > 0 && cmd.pose.position.z < MAX_HEIGHT ){
          cmd.pose.position.z -= 5;
          RCLCPP_INFO(this->get_logger(), "Increasing height: %f", cmd.pose.position.z);
          // button[4]: 왼쪽 범퍼
          // axex[7]: 왼쪽 스틱 y축 이동
        }
        if(!msg_joy->buttons[4] && msg_joy->axes[7] < 0 && cmd.pose.position.z > MIN_HEIGHT ){
          cmd.pose.position.z += 5;
          RCLCPP_INFO(this->get_logger(), "Decreasing height: %f", cmd.pose.position.z);
        }

        // set step height
        if(msg_joy->buttons[4] && msg_joy->axes[7] > 0 && cmd.gait_step.z < cmd.pose.position.z - MIN_HEIGHT){      
          cmd.gait_step.z += 5;
          RCLCPP_INFO(this->get_logger(), "Increasing step height: %f", cmd.gait_step.z);
        }
        if(msg_joy->buttons[4] && msg_joy->axes[7] < 0 && cmd.gait_step.z > 10 ){      
          cmd.gait_step.z -= 5;
          RCLCPP_INFO(this->get_logger(), "Decreasing step height: %f", cmd.gait_step.z);
        }

        // set eular angles
        if (!msg_joy->buttons[9] && !LJ_btn_sw){
          cmd.pose.orientation.x = -msg_joy->axes[0] * ROLL_RANGE;
          // axex[0]: 방향 스틱 x축 이동
          cmd.pose.orientation.y = msg_joy->axes[1] * PITCH_RANGE;
          // axex[1]: 방향 스틱 y축 이동
          cmd.pose.orientation.z = (msg_joy->axes[2] - msg_joy->axes[5])/2 * YAW_RANGE;
          // 왼쪽트리거: min, 오른쪽트리거: max
          RCLCPP_INFO(this->get_logger(), "Orientation -> Roll: %f, Pitch: %f, Yaw: %f",
            cmd.pose.orientation.x, cmd.pose.orientation.y, cmd.pose.orientation.z);
        }
      }
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg_rx) 
    {
      joy_state_to_joy_cmd(msg_rx);

      publisher_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<kaqu_msgs::msg::JoyCtrlCmds>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // Initialize ROS 2
  rclcpp::spin(std::make_shared<KaQuGamepadNode>());  // Start processing data from the node as well as the callbacks
  rclcpp::shutdown(); // Shutdown the node when finished
  return 0;
}
