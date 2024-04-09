import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from simple_pid import PID
import matplotlib.pyplot as plt

class Controller(Node):
    def __init__(self):
        super().__init__('distance_orientation_controller')

        # Initialize lists to store data
        self.time = []
        self.distance_errors = []
        self.orientation_errors = []
        self.right_turns = 0
        self.left_turns = 0

        # Initialize state flags
        self.distance_condition_met = False
        self.angle_condition_met = False

        # Subscribers
        self.subscription_distance = self.create_subscription(
            Vector3,
            'distance_variation',
            self.distance_callback,
            10)
        self.subscription_angles = self.create_subscription(
            Vector3,
            'angle_variation',
            self.angles_callback,
            10)

        # Publisher for robot control commands (e.g., velocity)
        self.publisher_velocity = self.create_publisher(Twist, 'cmd_vel', 10)

        # PID controllers
        self.pid_distance = PID(1.0, 0.2, 0.0, setpoint=6.0)
        self.pid_orientation = PID(0.5, 0.1, 0.5, setpoint=0.0)  # Assuming orientation is around Y-axis

        # Flag to indicate if the target has been reached
        self.target_reached = False

        # Flag to indicate if KeyboardInterrupt occurred
        self.keyboard_interrupt_occurred = False
        self.angle_correct = False
        self.distance_correct = False

    def distance_callback(self, msg):
        if not self.target_reached:
            distance_variation = msg.x  # Actual measured distance

            # Compute distance error (deviation from the target distance)
            distance_error = distance_variation - 13.0  # Target distance variation is 6.0m

            # Compute control signal using PID controller
            control_signal = self.pid_distance(distance_error)

            # Clamp control signal within speed limits
            control_signal = max(min(control_signal, 0.2), -0.2)

            # Log data
            self.time.append(len(self.time))
            self.distance_errors.append(distance_error)

            # Print distance and control signal
            print("Distance variation:", distance_variation)
            print("Control signal:", control_signal)

            # Create a Twist message and publish it
            twist = Twist()
            if control_signal < 0 or distance_error > 0.3:
                twist.linear.x = control_signal  # Negative speed for backward motion
                print("Moving forward:")
                print("Distance error:", distance_error)
                print("Distance variation:", distance_variation)
                self.distance_correct = (0 < distance_error < 0.3)
                
                if self.angle_correct and 0 < distance_error < 0.3:
                    print("Distance error within range (< 0.3):", distance_error)
                    print("Angle variation:", msg.y)  # Print angle variation
                    print("Target reached! Shutting down...")
                    self.target_reached = True
                    self.angle_correct = False
                    self.destroy_node()

                
            elif abs(distance_error < 0):
                self.distance_correct = False
                twist.linear.x = -control_signal  # Positive speed for forward motion
                print("Moving backward:")
                print("speed:", twist.linear.x)
                print("Distance error:", distance_error)
                print("Distance variation:", distance_variation)
            

            self.publisher_velocity.publish(twist)



    def angles_callback(self, msg):
        if not self.target_reached:
            angle_variation = msg.y  # Angle variation received from the subscribed topic

            # Initialize control signal
            control_signal = 0.0

            # Check if angle variation is within the desired range
            if 0 < abs(angle_variation) <= 0.055:
                twist_angular_z = 0.0  # No rotation needed

                # Log data
                self.orientation_errors.append(angle_variation)

                # Create a Twist message and publish it
                twist = Twist()
                twist.angular.z = twist_angular_z

                print("No rotation needed, angle variation:", angle_variation)
                self.angle_correct = True

                self.publisher_velocity.publish(twist)

            else:
                # Compute control signal using PID controller
                control_signal = self.pid_orientation(angle_variation)

                # Clamp control signal within speed limits
                control_signal = max(min(control_signal, 0.1), -0.1)

                # Determine direction of rotation based on the sign of control signal
                twist_angular_z = -control_signal  

                # Log data
                self.orientation_errors.append(angle_variation)

                # Create a Twist message and publish it
                twist = Twist()
                twist.angular.z = twist_angular_z
                
                if twist_angular_z < 0.0:
                    print(f"Turn left: {twist_angular_z}, angle variation: {angle_variation}")
                    self.left_turns += 1
                elif twist_angular_z > 0.0:
                    print(f"Turn right: {twist_angular_z}, angle variation: {angle_variation}")
                    self.right_turns += 1

                self.publisher_velocity.publish(twist)
                self.angle_correct = False
                self.check_target_reached()

    def check_target_reached(self):
        if self.distance_correct and self.angle_correct:
            print("Both distance and angle conditions are met. Target reached.")
            self.target_reached = True
            # Handle target reached logic, e.g., stop the robot, clean up, etc.
            self.destroy_node()
          

    def plot_data(self):
        plt.figure(figsize=(10, 6))

        plt.subplot(2, 1, 1)
        plt.plot(self.time, self.distance_errors, label='Distance Error')
        plt.title('Distance Error Over Time')
        plt.xlabel('Time')
        plt.ylabel('Distance Error')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(self.time, self.orientation_errors, label='Orientation Error')
        plt.title('Orientation Error Over Time')
        plt.xlabel('Time')
        plt.ylabel('Orientation Error')
        plt.legend()

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.keyboard_interrupt_occurred = True

    if controller.keyboard_interrupt_occurred:
        print("Total Right Turns:", controller.right_turns)
        print("Total Left Turns:", controller.left_turns)
    else:
        print("Total Right Turns:", controller.right_turns)
        print("Total Left Turns:", controller.left_turns)
        controller.plot_data()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
