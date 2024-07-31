# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import json
import os
import threading
from std_msgs.msg import String

class HumanoidRobotIntelligenceSystemCommand:
    def __init__(self, node):
        self.node = node
        self.current_ai_response = None
        self.process_thread = None
        self.process_lock = threading.Lock()
        self.stop_event = threading.Event()
        
        if ros_version == "1":
            self.done_msg_pub = rospy.Publisher('/humanoid_robot_intelligence_control_system/movement_done', String, queue_size=5)
            rospy.Subscriber('/ai_system_node_ai_response', String, self.ai_response_callback)
        elif ros_version == "2":
            self.done_msg_pub = self.node.create_publisher(String, '/humanoid_robot_intelligence_control_system/movement_done', 5)
            self.node.create_subscription(String, '/ai_system_node_ai_response', self.ai_response_callback, 10)

    def ai_response_callback(self, msg):
        self.process_input(msg.data, "ai_response")

    def process_input(self, data, input_type):
        done_msg = String()
        
        if not data:
            self.stop_processing()
            done_msg.data = f"{input_type}_process_fail"
            self.done_msg_pub.publish(done_msg)
            return

        self.stop_processing()
        
        try:
            self.stop_event.clear()
            self.process_thread = threading.Thread(target=self.process_data, args=(data, input_type))
            self.process_thread.start()
            done_msg.data = f"{input_type}_process_start"
        except Exception as e:
            self.log_error(f"Failed to process {input_type}: {e}")
            done_msg.data = f"{input_type}_process_fail"
        
        self.done_msg_pub.publish(done_msg)

    def process_data(self, data, input_type):
        with self.process_lock:
            if input_type == "ai_response":
                self.current_ai_response = data
                self.log_info(f"Processing AI response: {self.current_ai_response}")
                self.process_ai_response(self.current_ai_response)
            
            if not self.stop_event.is_set():
                self.log_info(f"Finished processing {input_type}: {data}")
                done_msg = String()
                done_msg.data = f"{input_type}_process_complete"
                self.done_msg_pub.publish(done_msg)

    def process_ai_response(self, response):
        try:
            response_dict = json.loads(response)
            action = response_dict.get('action', '')
            parameters = response_dict.get('parameters', {})

            if action == 'move':
                self.handle_move_action(parameters)
            elif action == 'speak':
                self.handle_speak_action(parameters)
            elif action == 'analyze':
                self.handle_analyze_action(parameters)
            else:
                self.log_warn(f"Unknown action: {action}")

        except json.JSONDecodeError:
            self.log_error(f"Invalid JSON in AI response: {response}")
        except Exception as e:
            self.log_error(f"Error processing AI response: {e}")

    def handle_move_action(self, parameters):
        direction = parameters.get('direction', '')
        distance = parameters.get('distance', 0)
        self.log_info(f"Handling move action: direction={direction}, distance={distance}")

    def handle_speak_action(self, parameters):
        text = parameters.get('text', '')
        self.log_info(f"Handling speak action: text='{text}'")

    def handle_analyze_action(self, parameters):
        target = parameters.get('target', '')
        method = parameters.get('method', '')
        self.log_info(f"Handling analyze action: target={target}, method={method}")

    def stop_processing(self):
        if self.process_thread and self.process_thread.is_alive():
            self.stop_event.set()
            self.process_thread.join(timeout=1)
            if self.process_thread.is_alive():
                self.log_warn("Failed to stop the processing thread gracefully")
            else:
                self.log_info("Processing stopped successfully")
        self.stop_event.clear()

    def log_info(self, message):
        if ros_version == "1":
            rospy.loginfo(message)
        elif ros_version == "2":
            self.node.get_logger().info(message)

    def log_warn(self, message):
        if ros_version == "1":
            rospy.logwarn(message)
        elif ros_version == "2":
            self.node.get_logger().warn(message)

    def log_error(self, message):
        if ros_version == "1":
            rospy.logerr(message)
        elif ros_version == "2":
            self.node.get_logger().error(message)

    def run(self):
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(self.node)

def main():
    global ros_version
    ros_version = os.getenv("ROS_VERSION", "1")

    if ros_version == "1":
        import rospy
        rospy.init_node('humanoid_robot_intelligence_control_system')
        system = HumanoidRobotIntelligenceSystem(None)
    elif ros_version == "2":
        import rclpy
        rclpy.init()
        node = rclpy.create_node('humanoid_robot_intelligence_control_system')
        system = HumanoidRobotIntelligenceSystem(node)
    else:
        print("Unsupported ROS version")
        return

    try:
        print("Humanoid Robot Intelligence Control System is running. Ctrl+C to exit.")
        system.run()
    except KeyboardInterrupt:
        print("Shutting down Humanoid Robot Intelligence Control System.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        if ros_version == "2":
            rclpy.shutdown()

if __name__ == '__main__':
    main()
