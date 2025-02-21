import signal
import sys
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import math
import rclpy
from rclpy.node import Node
from multi_crane_msg.msg import MultiCraneMsg, TowerCraneMsg, LuffingJibCraneMsg  # Replace with your package name


class TowerCrane:
    def __init__(self, tower_crane_msg):
        """
        Initialize a tower crane.
        
        """
        self.crane_id = tower_crane_msg.crane_id
        self.crane_type = tower_crane_msg.crane_type
        self.crane_x = tower_crane_msg.crane_x
        self.crane_y = tower_crane_msg.crane_y
        self.crane_z = tower_crane_msg.crane_z
        self.boom_length = tower_crane_msg.boom_length
        
        self.boom_angle  = tower_crane_msg.boom_angle
        self.trolley_radius = tower_crane_msg.trolley_radius
        self.hook_height = tower_crane_msg.hook_height

        self.boom_end_x = None
        self.boom_end_y = None
        self.boom_end_z = None

        self.trolley_x = None
        self.trolley_y = None
        self.trolley_z = None

        self.hook_x = None
        self.hook_y = None
        self.hook_z = None

        self.cylindricalCoor2cartesianCoor()
    
    def cylindricalCoor2cartesianCoor(self):
        """
        xx

        :id, type, crane_x, crane_y, crane_z, boom_length,boom_horAngle,boom_verAngle,hook_height
        """
        self.boom_end_x = self.crane_x  + self.boom_length*math.cos(self.boom_angle)
        self.boom_end_y = self.crane_y  + self.boom_length*math.sin(self.boom_angle)
        self.boom_end_z = self.crane_z

        self.trolley_x = self.crane_x  + self.trolley_radius*math.cos(self.boom_angle)
        self.trolley_y = self.crane_y  + self.trolley_radius*math.sin(self.boom_angle)
        self.trolley_z = self.crane_z

        self.hook_x = self.trolley_x
        self.hook_y = self.trolley_y
        # self.hook_z = self.hook_height
        self.hook_z = self.trolley_z - self.hook_height

    def tower_crane_update_state(self, tower_crane_msg):
        """
        update states for tower crane.
        """
        self.crane_id = tower_crane_msg.crane_id
        self.crane_type = tower_crane_msg.crane_type
        self.crane_x = tower_crane_msg.crane_x
        self.crane_y = tower_crane_msg.crane_y
        self.crane_z = tower_crane_msg.crane_z
        self.boom_length = tower_crane_msg.boom_length
        
        self.boom_angle  = tower_crane_msg.boom_angle
        self.trolley_radius = tower_crane_msg.trolley_radius
        self.hook_height = tower_crane_msg.hook_height
        
        self.cylindricalCoor2cartesianCoor()

    def check_collision(self, other_crane):
        """
        Check for collision with another crane.
        
        :param other_crane: Another TowerCrane object.
        :return: True if collision detected, False otherwise.
        """
        pass


class LuffingJibCrane:
    def __init__(self, luffing_jib_crane_msg):
        """
        Initialize a luffing jib crane.
        """
        self.crane_id = luffing_jib_crane_msg.crane_id
        self.crane_type = luffing_jib_crane_msg.crane_type
        self.crane_x = luffing_jib_crane_msg.crane_x
        self.crane_y = luffing_jib_crane_msg.crane_y
        self.crane_z = luffing_jib_crane_msg.crane_z
        self.boom_length = luffing_jib_crane_msg.boom_length
        
        self.boom_horAngle = luffing_jib_crane_msg.boom_hor_angle
        self.boom_verAngle = luffing_jib_crane_msg.boom_ver_angle
        self.hook_height = luffing_jib_crane_msg.hook_height

        self.boom_end_x = None
        self.boom_end_y = None
        self.boom_end_z = None

        self.hook_x = None
        self.hook_y = None
        self.hook_z = None

        self.cylindricalCoor2cartesianCoor()
    
    def cylindricalCoor2cartesianCoor(self):
        """
        xx

        :
        """
        self.boom_end_x = self.crane_x  + (self.boom_length*math.cos(self.boom_verAngle))*math.cos(self.boom_horAngle)
        self.boom_end_y = self.crane_y  + (self.boom_length*math.cos(self.boom_verAngle))*math.sin(self.boom_horAngle)
        self.boom_end_z = self.crane_z  + self.boom_length*math.sin(self.boom_verAngle)

        self.hook_x = self.boom_end_x  
        self.hook_y = self.boom_end_y  
        self.hook_z = self.boom_end_z - self.hook_height
    
    def lj_crane_update_state(self, luffing_jib_crane_msg):
        """
        """
        self.crane_id = luffing_jib_crane_msg.crane_id
        self.crane_type = luffing_jib_crane_msg.crane_type
        self.crane_x = luffing_jib_crane_msg.crane_x
        self.crane_y = luffing_jib_crane_msg.crane_y
        self.crane_z = luffing_jib_crane_msg.crane_z
        self.boom_length = luffing_jib_crane_msg.boom_length
        
        self.boom_horAngle = luffing_jib_crane_msg.boom_hor_angle
        self.boom_verAngle = luffing_jib_crane_msg.boom_ver_angle
        self.hook_height = luffing_jib_crane_msg.hook_height

        self.cylindricalCoor2cartesianCoor()


class CraneSwarm:
    def __init__(self):
        """
        Initialize the CraneSwarm.
        """
        self.cranes_id = []
        self.cranes = []
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(18, 6))
        self.ax1 = self.fig.add_subplot(131, projection='3d')  # 3D View
        self.ax2 = self.fig.add_subplot(132)  # Front View (2D)
        self.ax3 = self.fig.add_subplot(133)  # Top View (2D)
        self.update_visualization()

    def add_crane(self, crane):
        """
        Add a crane to the crane_swarm.
        
        :param crane: TowerCrane object.
        """
        self.cranes.append(crane)
        self.cranes_id.append(crane.crane_id)
    
    def update_crane_state(self, crane_msg):
        """
        Update the state of a specific crane and refresh the visualization.
        """
        for crane in self.cranes:
            if crane.crane_id == crane_msg.crane_id:
                if crane.crane_type == "towerCrane":
                    crane.tower_crane_update_state(crane_msg)
                elif crane.crane_type == "luffingJibCrane":
                    crane.lj_crane_update_state(crane_msg)
                break
        self.update_visualization()

    def update_visualization(self):
        """
        Clear and redraw the plots.
        """
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        self.visualize_3d(self.ax1)
        self.visualize_front_view(self.ax2)
        self.visualize_top_view(self.ax3)

        plt.draw()
        plt.pause(0.1)

    def visualize_tower_crane_3D(self,crane,ax):
        # Plot the base of the crane
        ax.scatter(crane.crane_x, crane.crane_y, 0, color='blue')
        # Plot the tower of the crane
        ax.plot([crane.crane_x, crane.crane_x], [crane.crane_y, crane.crane_y], [0, crane.crane_z], color='red')
        # Plot the boom of the crane
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_y, crane.boom_end_y], [crane.crane_z, crane.boom_end_z], color='green')
        ax.text(crane.crane_x, crane.crane_y, crane.crane_z, f"Crane {crane.crane_id}", fontsize=8)
        # Plot the trolley of the crane
        ax.scatter(crane.trolley_x, crane.trolley_y, crane.trolley_z, color='orange')
        # Plot the hook rope of the crane
        ax.plot([crane.trolley_x, crane.hook_x], [crane.trolley_y, crane.hook_y], [crane.trolley_z, crane.hook_z], color='purple',linestyle='--')
        # Plot the hook of the crane
        ax.scatter(crane.hook_x, crane.hook_y, crane.hook_z, color='purple')
    
    def visualize_tower_crane_front_view(self,crane,ax):
        # Plot the base of the crane
        ax.scatter(crane.crane_x, 0, color='blue')
        # Plot the height of the crane (vertical line)
        ax.plot([crane.crane_x, crane.crane_x], [0, crane.crane_z], color='red')
        # Plot the boom (horizontal line at the top of the crane)
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_z, crane.boom_end_z], color='green')
        ax.text(crane.crane_x, crane.crane_z, f"Crane {crane.crane_id}", fontsize=8)
        # Plot the trolley position on the boom
        ax.scatter(crane.trolley_x, crane.trolley_z, color='orange')
        # Plot the hook and rope (vertical line from the trolley to the hook height)
        ax.plot([crane.trolley_x, crane.hook_x], [crane.trolley_z, crane.hook_z], color='purple', linestyle='--')
        # Plot the hook 
        ax.scatter(crane.hook_x, crane.hook_z, color='purple')

    def visualize_luffing_jib_crane_3D(self,crane,ax):
        # Plot the base of the crane
        ax.scatter(crane.crane_x, crane.crane_y, 0, color='blue', label=f"Crane {crane.crane_id}")
        # Plot the tower of the crane
        ax.plot([crane.crane_x, crane.crane_x], [crane.crane_y, crane.crane_y], [0, crane.crane_z], color='red')
        ax.text(crane.crane_x, crane.crane_y, crane.crane_z, f"Crane {crane.crane_id}", fontsize=8)
        # Plot the boom of the crane
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_y, crane.boom_end_y], [crane.crane_z, crane.boom_end_z], color='green')
        # Plot the hook rope of the crane
        ax.plot([crane.boom_end_x, crane.hook_x], [crane.boom_end_y, crane.hook_y], [crane.boom_end_z, crane.hook_z], color='purple',linestyle='--')
        # Plot the hook of the crane
        ax.scatter(crane.hook_x, crane.hook_y, crane.hook_z, color='purple')

    def visualize_lj_crane_front_view(self,crane,ax):
        # Plot the base of the crane
        ax.scatter(crane.crane_x, 0, color='blue', label=f"Crane {crane.crane_id}")
        # Plot the tower of the crane
        ax.plot([crane.crane_x, crane.crane_x], [0, crane.crane_z], color='red')
        # Plot the boom of the crane
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_z, crane.boom_end_z], color='green')
        ax.text(crane.crane_x, crane.crane_z, f"Crane {crane.crane_id}", fontsize=8)
        # Plot the hook rope of the crane
        ax.plot([crane.boom_end_x, crane.hook_x], [crane.boom_end_z, crane.hook_z], color='purple',linestyle='--')
        # Plot the hook of the crane
        ax.scatter(crane.hook_x, crane.hook_z, color='purple')

    def visualize_lj_crane_top_view(self,crane,ax):
        boom_length_projection = crane.boom_length*math.cos(crane.boom_verAngle)

        ax.scatter(crane.crane_x, crane.crane_y, color='blue')
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_y, crane.boom_end_y], color='green')
        circle = plt.Circle((crane.crane_x, crane.crane_y), boom_length_projection, color='red', linestyle='--',fill=False)
        plt.gca().add_patch(circle)
        plt.text(crane.crane_x, crane.crane_y, f"Crane {crane.crane_id}", fontsize=8, ha='center')

    def visualize_tower_crane_top_view(self,crane,ax):
        ax.scatter(crane.crane_x, crane.crane_y, color='blue')
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_y, crane.boom_end_y], color='green')
        circle = plt.Circle((crane.crane_x, crane.crane_y), crane.boom_length, color='red', linestyle='--',fill=False)
        plt.gca().add_patch(circle)
        plt.text(crane.crane_x, crane.crane_y, f"Crane {crane.crane_id}", fontsize=8, ha='center')


    def visualize_3d(self,ax):
        """
        Visualize all cranes in a 3D plot with booms.
        """
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')

        for crane in self.cranes:
            if crane.crane_type == "towerCrane":
                self.visualize_tower_crane_3D(crane,ax)
            elif crane.crane_type == "luffingJibCrane":
                self.visualize_luffing_jib_crane_3D(crane,ax)

        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_zlim(0, 100)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_zlabel("Height (m)")
        ax.set_title("3D View")


    def visualize_front_view(self,ax):
        """
        Visualize all cranes in a 2D front view, including the boom, trolley, and hook.
        """
        for crane in self.cranes:
            if crane.crane_type == "towerCrane":
                self.visualize_tower_crane_front_view(crane,ax)
            elif crane.crane_type == "luffingJibCrane":
                self.visualize_lj_crane_front_view(crane,ax)

        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Height (m)")
        ax.set_title("Front View (2D)")


    def visualize_top_view(self,ax):
        """
        Visualize all cranes in a 2D plot.
        """
        for crane in self.cranes:
            if crane.crane_type == "towerCrane":
                self.visualize_tower_crane_top_view(crane,ax)
            elif crane.crane_type == "luffingJibCrane":
                self.visualize_lj_crane_top_view(crane,ax)
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title("Top View (2D)")

    
    # def visualize_all(self):
    #     """
    #     Visualize all three views (3D, front view, top view) in the same figure.
    #     """
    #     # 3D View
    #     self.visualize_3d(self.ax1)

    #     # Front View (2D)
    #     self.visualize_front_view(self.ax2)

    #     # Top View (2D)
    #     self.visualize_top_view(self.ax3)

    #     plt.tight_layout()
    #     plt.show(block=False)

    def check_all_collisions(self):
        """
        Check for collisions between all cranes.
        """
        for i in range(len(self.cranes)):
            for j in range(i + 1, len(self.cranes)):
                if self.cranes[i].check_collision(self.cranes[j]):
                    print(f"Collision detected between Crane {self.cranes[i].crane_id} and Crane {self.cranes[j].crane_id}")

class MultiCraneSubscriber(Node):

    def __init__(self):
        super().__init__('multi_crane_subscriber')
        self.subscription = self.create_subscription(
            MultiCraneMsg,
            'multi_crane',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        """
        global crane_swarm

        # parse TowerCraneMsg array
        for tower_crane_msg in msg.tower_crane_msgs:
            self.get_logger().info(
                f'Tower Crane ID: {tower_crane_msg.crane_id}, '
                f'Type: {tower_crane_msg.crane_type}, '
                f'Position: ({tower_crane_msg.crane_x}, {tower_crane_msg.crane_y}, {tower_crane_msg.crane_z}), '
                f'Boom Length: {tower_crane_msg.boom_length}, '
                f'Boom Angle: {tower_crane_msg.boom_angle}, '
                f'Trolley Radius: {tower_crane_msg.trolley_radius}, '
                f'Hook Height: {tower_crane_msg.hook_height}'
            )
            if tower_crane_msg.crane_id in crane_swarm.cranes_id:
                crane_swarm.update_crane_state(tower_crane_msg)
            else:
                # create a crane if there doesn't exists 
                new_tower_crane = TowerCrane(tower_crane_msg)
                crane_swarm.add_crane(new_tower_crane)
            
        # parse LuffingJibCraneMsg array
        for lj_crane_msg in msg.luffing_jib_crane_msgs:
            # for debug
            self.get_logger().info(
                f'Luffing Jib Crane ID: {lj_crane_msg.crane_id}, '
                f'Type: {lj_crane_msg.crane_type}, '
                f'Position: ({lj_crane_msg.crane_x}, {lj_crane_msg.crane_y}, {lj_crane_msg.crane_z}), '
                f'Boom Length: {lj_crane_msg.boom_length}, '
                f'Boom Horizontal Angle: {lj_crane_msg.boom_hor_angle}, '
                f'Boom Vertical Angle: {lj_crane_msg.boom_ver_angle}, '
                f'Hook Height: {lj_crane_msg.hook_height}'
            )
            if lj_crane_msg.crane_id in crane_swarm.cranes_id:
                crane_swarm.update_crane_state(lj_crane_msg)
            else:
                # create a crane if there doesn't exists 
                new_lj_crane = LuffingJibCrane(lj_crane_msg)
                crane_swarm.add_crane(new_lj_crane) 
            

        
def signal_handler(sig, frame):
    """
    Handle the signal (e.g., Ctrl+C) to gracefully exit the program.
    """
    print("\nProgram terminated by user. Exiting gracefully...")
    plt.close('all')  # Close all Matplotlib figures
    plt.ioff()        # Turn off interactive mode
    sys.exit(0)       # Exit the program

def main(args=None):
    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create a towerCrane message
    tower_crane_msg = TowerCraneMsg()
    tower_crane_msg.crane_id = 1
    tower_crane_msg.crane_type = "towerCrane"
    tower_crane_msg.crane_x = 20.0
    tower_crane_msg.crane_y = 30.0
    tower_crane_msg.crane_z = 50.0
    tower_crane_msg.boom_length = 25.0
    tower_crane_msg.boom_angle = 45.0 # Dynamically changing
    tower_crane_msg.trolley_radius = 16.3  # Dynamically changing
    tower_crane_msg.hook_height = 10.7  # Dynamically changing

    # Create a LuffingJibCrane message
    luffing_jib_crane_msg = LuffingJibCraneMsg()
    luffing_jib_crane_msg.crane_id = 4
    luffing_jib_crane_msg.crane_type = "luffingJibCrane"
    luffing_jib_crane_msg.crane_x = 66.0
    luffing_jib_crane_msg.crane_y = 52.0
    luffing_jib_crane_msg.crane_z = 45.0
    luffing_jib_crane_msg.boom_length = 18.4
    luffing_jib_crane_msg.boom_hor_angle = 50.0 # Dynamically changing
    luffing_jib_crane_msg.boom_ver_angle = 60.0 # Dynamically changing
    luffing_jib_crane_msg.hook_height = 10.2  # Dynamically changing

    # Create cranes
    crane1 = TowerCrane(tower_crane_msg)
    crane2 = LuffingJibCrane(luffing_jib_crane_msg)
   
    # Add cranes to crane_swarm
    global crane_swarm
    crane_swarm = CraneSwarm()
    crane_swarm.add_crane(crane1)
    crane_swarm.add_crane(crane2)
    crane_swarm.update_visualization()

    # create a ros2 node for subscriber. subscriber contains the information
    # to update crane states
    rclpy.init(args=args)
    multi_crane_subscriber = MultiCraneSubscriber()
    rclpy.spin(multi_crane_subscriber)
    # Destroy the node explicitly (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_crane_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
