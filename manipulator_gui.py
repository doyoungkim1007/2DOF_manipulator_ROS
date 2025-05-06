import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from manipul_interfaces.srv import SetFloat64Array
import numpy as np
import math

class ManipulatorGUI(Node):
    def __init__(self):
        super().__init__('manipulator_gui')
        self.create_subscription(Float64MultiArray, 'joint_angles', self.joint_angles_callback, 10)
        self.cli = self.create_client(SetFloat64Array, 'set_lengths')
        
        self.joint_angles_pub = self.create_publisher(Float64MultiArray, 'joint_angles', 10)  # 퍼블리셔 초기화
        
        self.root = tk.Tk()
        self.root.title("2-DOF Manipulator")
        self.canvas = tk.Canvas(self.root, width=500, height=500)
        self.canvas.pack()
        
        self.joint_angles = [0.0, 0.0]
        self.link_lengths = [100.0, 100.0]
        
        self.end_effector = self.forward_kinematics(self.joint_angles)
        self.draw_manipulator()
        
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<Motion>", self.on_mouse_move)
        
        self.create_link_length_inputs()
        
        self.root.mainloop()

    def create_link_length_inputs(self):
        self.length1_label = tk.Label(self.root, text="Link 1 Length:")
        self.length1_label.pack()
        self.length1_entry = tk.Entry(self.root)
        self.length1_entry.pack()
        self.length1_entry.insert(0, str(self.link_lengths[0]))

        self.length2_label = tk.Label(self.root, text="Link 2 Length:")
        self.length2_label.pack()
        self.length2_entry = tk.Entry(self.root)
        self.length2_entry.pack()
        self.length2_entry.insert(0, str(self.link_lengths[1]))

        self.update_button = tk.Button(self.root, text="Update Lengths", command=self.update_lengths)
        self.update_button.pack()

    def update_lengths(self):
        try:
            length1 = float(self.length1_entry.get())
            length2 = float(self.length2_entry.get())
            self.link_lengths = [length1, length2]
            self.send_request()
            self.draw_manipulator()
        except ValueError:
            print("Invalid input for link lengths. Please enter numeric values.")

    def send_request(self):
        req = SetFloat64Array.Request()
        req.data = self.link_lengths
        self.future = self.cli.call_async(req)

    def draw_manipulator(self):
        if not hasattr(self, 'canvas') or not self.canvas.winfo_exists():
            return

        self.canvas.delete("all")

        base_x, base_y = 250, 250
        joint1_x = base_x + self.link_lengths[0] * math.cos(self.joint_angles[0])
        joint1_y = base_y + self.link_lengths[0] * math.sin(self.joint_angles[0])
        end_effector_x = joint1_x + self.link_lengths[1] * math.cos(self.joint_angles[0] + self.joint_angles[1])
        end_effector_y = joint1_y + self.link_lengths[1] * math.sin(self.joint_angles[0] + self.joint_angles[1])

        self.canvas.create_line(base_x, base_y, joint1_x, joint1_y, fill="#00EBFF", width=5)
        self.canvas.create_line(joint1_x, joint1_y, end_effector_x, end_effector_y, fill="#00EBFF", width=5)

        self.canvas.create_oval(base_x-5, base_y-5, base_x+5, base_y+5, fill="black")
        self.canvas.create_oval(joint1_x-5, joint1_y-5, joint1_x+5, joint1_y+5, fill="black")
        self.canvas.create_oval(end_effector_x-5, end_effector_y-5, end_effector_x+5, end_effector_y+5, fill="black")

        self.end_effector = (end_effector_x, end_effector_y)

    def forward_kinematics(self, joint_angles):
        base_x, base_y = 250, 250
        joint1_x = base_x + self.link_lengths[0] * math.cos(joint_angles[0])
        joint1_y = base_y + self.link_lengths[0] * math.sin(joint_angles[0])
        end_effector_x = joint1_x + self.link_lengths[1] * math.cos(joint_angles[0] + joint_angles[1])
        end_effector_y = joint1_y + self.link_lengths[1] * math.sin(joint_angles[0] + joint_angles[1])
        return end_effector_x, end_effector_y

    def joint_angles_callback(self, msg):
        self.joint_angles = msg.data
        self.draw_manipulator()

    def on_click(self, event):
        target_x, target_y = event.x, event.y
        try:
            target_angles = self.inverse_kinematics(target_x, target_y)
            self.animate_movement(target_angles)
        except ValueError as e:
            self.show_warning(str(e))

    def animate_movement(self, target_angles):
        steps = 100
        current_angles = np.array(self.joint_angles, dtype=np.float64)
        target_angles = np.array(target_angles, dtype=np.float64)
        delta_angles = (target_angles - current_angles) / steps

        for _ in range(steps):
            current_angles += delta_angles
            self.joint_angles = current_angles.tolist()
            self.publish_joint_angles()
            self.draw_manipulator()
            self.root.update_idletasks()
            self.root.after(10)

    def publish_joint_angles(self):
        msg = Float64MultiArray()
        msg.data = self.joint_angles
        self.joint_angles_pub.publish(msg)

    def inverse_kinematics(self, target_x, target_y):
        base_x, base_y = 250, 250
        dx = target_x - base_x
        dy = target_y - base_y
        distance = np.sqrt(dx**2 + dy**2)

        if distance > sum(self.link_lengths):
            raise ValueError("Target is out of reach")

        cos_angle2 = (dx**2 + dy**2 - self.link_lengths[0]**2 - self.link_lengths[1]**2) / (2 * self.link_lengths[0] * self.link_lengths[1])

        if cos_angle2 < -1 or cos_angle2 > 1:
            raise ValueError("Target is out of reach due to invalid cos_angle2 value")

        sin_angle2 = np.sqrt(1 - cos_angle2**2)
        theta2 = math.atan2(sin_angle2, cos_angle2)

        k1 = self.link_lengths[0] + self.link_lengths[1] * cos_angle2
        k2 = self.link_lengths[1] * sin_angle2
        theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)

        return [theta1, theta2]

    def show_warning(self, message):
        warning_popup = tk.Toplevel(self.root)
        warning_popup.title("Warning")
        tk.Label(warning_popup, text=message).pack()
        tk.Button(warning_popup, text="OK", command=warning_popup.destroy).pack()
        warning_popup.update_idletasks()
        width = warning_popup.winfo_width()
        height = warning_popup.winfo_height()
        # 현재 창의 중앙 좌표 계산
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - (width // 2)
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - (height // 2)
        warning_popup.geometry(f'{width}x{height}+{x}+{y}')


    def on_mouse_move(self, event):
        self.canvas.delete("cursor_position")
        self.canvas.create_text(event.x, event.y, text=f"({event.x}, {event.y})", tag="cursor_position")

def main(args=None):
    rclpy.init(args=args)
    gui_node = ManipulatorGUI()
    rclpy.spin(gui_node)
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
