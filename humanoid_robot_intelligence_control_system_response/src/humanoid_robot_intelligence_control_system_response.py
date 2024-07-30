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

import rospy
from std_msgs.msg import String
import threading
import time

class HumanoidRobotIntelligenceSystem:
    def __init__(self):
        rospy.init_node('humanoid_robot_intelligence_control_system')
        
        self.current_sound = None
        self.current_ai_response = None
        self.process_thread = None
        self.process_lock = threading.Lock()
        self.stop_event = threading.Event()
        
        self.done_msg_pub = rospy.Publisher('/humanoid_robot_intelligence_control_system/movement_done', String, queue_size=5)
        rospy.Subscriber('/ai_system_node_response', String, self.ai_response_callback)

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
            rospy.logerr(f"Failed to process {input_type}: {e}")
            done_msg.data = f"{input_type}_process_fail"
        
        self.done_msg_pub.publish(done_msg)

    def process_data(self, data, input_type):
        with self.process_lock:
            if input_type == "ai_response":
                self.current_ai_response = data
                rospy.loginfo(f"Processing AI response: {self.current_ai_response}")
                self.simulate_processing("AI response processing")
            
            if not self.stop_event.is_set():
                rospy.loginfo(f"Finished processing {input_type}: {data}")
                done_msg = String()
                done_msg.data = f"{input_type}_process_complete"
                self.done_msg_pub.publish(done_msg)

    def simulate_processing(self, process_name):
        total_time = 5
        steps = 10
        for i in range(steps):
            if self.stop_event.is_set():
                rospy.loginfo(f"{process_name} interrupted at {(i+1)/steps*100}%")
                return
            time.sleep(total_time / steps)
            rospy.loginfo(f"{process_name} progress: {(i+1)/steps*100}%")

    def stop_processing(self):
        if self.process_thread and self.process_thread.is_alive():
            self.stop_event.set()
            self.process_thread.join(timeout=1)
            if self.process_thread.is_alive():
                rospy.logwarn("Failed to stop the processing thread gracefully")
            else:
                rospy.loginfo("Processing stopped successfully")
        self.stop_event.clear()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    system = HumanoidRobotIntelligenceSystem()
    system.run()
