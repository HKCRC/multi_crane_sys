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
    def __init__(self, id, type, crane_x, crane_y, crane_z, boom_length,boom_angle,trolley_radius,hook_height):
        """
        Initialize a tower crane.
        
        :param id: Unique identifier for the crane.
        :param type: type for the crane.
        :param crane_x: X-coordinate of the crane's base.
        :param crane_y: Y-coordinate of the crane's base.
        :param crane_z: Z-coordinate of the crane's base.
        :param boom_length: boom_length of the crane.
        :param boom_angle : boom_angle  of the crane.
        :param trolley_radius: trolley_radius of the crane.
        :param hook_height: hook_height of the crane.
        """
        self.crane_id = id
        self.crane_type = type
        self.crane_x = crane_x
        self.crane_y = crane_y
        self.crane_z = crane_z
        self.boom_length = boom_length
        
        self.boom_angle  = boom_angle
        self.trolley_radius = trolley_radius
        self.hook_height = hook_height

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

        :
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

    def update_state(self, boom_angle=None, trolley_radius=None, hook_height=None):
        """
        update states for tower crane.
        """
        if boom_angle is not None:
            self.boom_angle = boom_angle
        if trolley_radius is not None:
            self.trolley_radius = trolley_radius
        if hook_height is not None:
            self.hook_height = hook_height
        self.cylindricalCoor2cartesianCoor()

    def check_collision(self, other_crane):
        """
        Check for collision with another crane.
        
        :param other_crane: Another TowerCrane object.
        :return: True if collision detected, False otherwise.
        """
        pass


class LuffingJibCrane:
    def __init__(self, id, type, crane_x, crane_y, crane_z, boom_length,boom_horAngle,boom_verAngle,hook_height):
        """
        Initialize a luffing jib crane.
        
        :param id: Unique identifier for the crane.
        :param type: type for the crane.
        :param crane_x: X-coordinate of the crane's base.
        :param crane_y: Y-coordinate of the crane's base.
        :param crane_z: Z-coordinate of the crane's base.
        :param boom_length: boom_length of the crane.
        """
        self.crane_id = id
        self.crane_type = type
        self.crane_x = crane_x
        self.crane_y = crane_y
        self.crane_z = crane_z
        self.boom_length = boom_length
        
        self.boom_horAngle = boom_horAngle
        self.boom_verAngle = boom_verAngle
        self.hook_height = hook_height

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
    
    def update_state(self, boom_horAngle=None, boom_verAngle=None, hook_height=None):
        """
        """
        if boom_horAngle is not None:
            self.boom_horAngle = boom_horAngle
        if boom_verAngle is not None:
            self.boom_verAngle = boom_verAngle
        if hook_height is not None:
            self.hook_height = hook_height
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
        self.visualize_all()

    def add_crane(self, crane):
        """
        Add a crane to the crane_swarm.
        
        :param crane: TowerCrane object.
        """
        self.cranes.append(crane)
        self.cranes_id.append(crane.crane_id)
    
    def update_crane_state(self, crane_id, **kwargs):
        """
        Update the state of a specific crane and refresh the visualization.
        """
        for crane in self.cranes:
            if crane.crane_id == crane_id:
                crane.update_state(**kwargs)
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
        # boom_length_projection = crane.boom_length*math.cos(crane.boom_verAngle)

        ax.scatter(crane.crane_x, crane.crane_y, color='blue')
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_y, crane.boom_end_y], color='green')
        circle = plt.Circle((crane.crane_x, crane.crane_y), crane.boom_length, color='red', linestyle='--',fill=False)
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

    
    def visualize_all(self):
        """
        Visualize all three views (3D, front view, top view) in the same figure.
        """
        # 3D View
        self.visualize_3d(self.ax1)

        # Front View (2D)
        self.visualize_front_view(self.ax2)

        # Top View (2D)
        self.visualize_top_view(self.ax3)

        plt.tight_layout()
        plt.show(block=False)

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
        for tower_crane in msg.tower_crane_msgs:
            self.get_logger().info(
                f'Tower Crane ID: {tower_crane.crane_id}, '
                f'Type: {tower_crane.crane_type}, '
                f'Position: ({tower_crane.crane_x}, {tower_crane.crane_y}, {tower_crane.crane_z}), '
                f'Boom Length: {tower_crane.boom_length}, '
                f'Boom Angle: {tower_crane.boom_angle}, '
                f'Trolley Radius: {tower_crane.trolley_radius}, '
                f'Hook Height: {tower_crane.hook_height}'
            )
            if tower_crane.crane_id in crane_swarm.cranes_id:
                crane_swarm.update_crane_state(tower_crane.crane_id, boom_angle=tower_crane.boom_angle, \
                trolley_radius=tower_crane.trolley_radius, hook_height=tower_crane.hook_height)
            else:
                # create a crane if there doesn't exists 
                new_tower_crane = TowerCrane(id=tower_crane.crane_id, type = tower_crane.crane_type, \
                          crane_x= tower_crane.crane_x, crane_y=tower_crane.crane_y, crane_z=tower_crane.crane_z,  \
                          boom_length=tower_crane.boom_length, boom_angle= tower_crane.boom_angle, \
                          trolley_radius= tower_crane.trolley_radius, hook_height =tower_crane.hook_height)
                crane_swarm.add_crane(new_tower_crane)
            


        # parse LuffingJibCraneMsg array
        for luffing_jib_crane in msg.luffing_jib_crane_msgs:
            # for debug
            self.get_logger().info(
                f'Luffing Jib Crane ID: {luffing_jib_crane.crane_id}, '
                f'Type: {luffing_jib_crane.crane_type}, '
                f'Position: ({luffing_jib_crane.crane_x}, {luffing_jib_crane.crane_y}, {luffing_jib_crane.crane_z}), '
                f'Boom Length: {luffing_jib_crane.boom_length}, '
                f'Boom Horizontal Angle: {luffing_jib_crane.boom_hor_angle}, '
                f'Boom Vertical Angle: {luffing_jib_crane.boom_ver_angle}, '
                f'Hook Height: {luffing_jib_crane.hook_height}'
            )
            if luffing_jib_crane.crane_id in crane_swarm.cranes_id:
                crane_swarm.update_crane_state(luffing_jib_crane.crane_id, boom_horAngle=luffing_jib_crane.boom_hor_angle, \
                    boom_verAngle=luffing_jib_crane.boom_ver_angle, hook_height=luffing_jib_crane.hook_height)
            else:
                # create a crane if there doesn't exists 
                id = luffing_jib_crane.crane_id
                type = luffing_jib_crane.crane_type
                crane_x = luffing_jib_crane.crane_x
                crane_y =luffing_jib_crane.crane_y
                crane_z = luffing_jib_crane.crane_z
                boom_length = luffing_jib_crane.boom_length
                boom_hor_angle = luffing_jib_crane.boom_hor_angle
                boom_ver_angle = luffing_jib_crane.boom_ver_angle
                hook_height = luffing_jib_crane.hook_height

                new_lj_crane = LuffingJibCrane(id=id, type = type, crane_x= crane_x, crane_y=crane_y, crane_z=crane_z,  \
                          boom_length = boom_length, boom_horAngle= boom_hor_angle, \
                          boom_verAngle= boom_ver_angle, hook_height =hook_height)
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

    # Create cranes
    crane1 = TowerCrane(id=1, type = 'towerCrane', crane_x=20, crane_y=30, crane_z=50,  boom_length=25, boom_angle= math.radians(30), trolley_radius= 5, hook_height =5)
    crane2 = TowerCrane(id=2, type = 'towerCrane', crane_x=40, crane_y=50, crane_z=40,  boom_length=15, boom_angle= math.radians(60), trolley_radius= 5, hook_height =5)
    crane3 = LuffingJibCrane(id=3, type = 'luffingJibCrane', crane_x=60, crane_y=20, crane_z=20,  boom_length=20, boom_horAngle= math.radians(30), boom_verAngle= math.radians(60), hook_height =15)

    # Add cranes to crane_swarm
    global crane_swarm
    crane_swarm = CraneSwarm()
    crane_swarm.add_crane(crane1)
    crane_swarm.add_crane(crane2)
    crane_swarm.add_crane(crane3)
    crane_swarm.visualize_all()

    ##  munual input to test
    # while True:
    #     try:
    #         data_source = str(input("Choose visual collision data source (keyboard or ros): "))
    #         if data_source == "ros":
    #             # create a ros2 node for subscriber. subscriber contains the information
    #             # to update crane states
    #             rclpy.init(args=args)
    #             multi_crane_subscriber = MultiCraneSubscriber()
    #             rclpy.spin(multi_crane_subscriber)
    #             # Destroy the node explicitly (optional - otherwise it will be done automatically
    #             # when the garbage collector destroys the node object)
    #             multi_crane_subscriber.destroy_node()
    #             rclpy.shutdown()

    #         elif data_source == "keyboard":
    #             crane_id = int(input(f"Enter crane ID to update ({crane_swarm.cranes_id}): "))
    #             for crane in crane_swarm.cranes:
    #                 if crane.crane_id == crane_id:
    #                     if crane.crane_type == 'towerCrane':
    #                         boom_angle = math.radians(float(input("Enter boom angle (degrees): ")))
    #                         trolley_radius = float(input("Enter trolley radius: "))
    #                         hook_height = float(input("Enter hook height: "))
    #                         crane_swarm.update_crane_state(crane_id, boom_angle=boom_angle, trolley_radius=trolley_radius, hook_height=hook_height)
    #                     elif crane.crane_type == 'luffingJibCrane':
    #                         boom_horAngle = math.radians(float(input("Enter boom horizontal angle (degrees): ")))
    #                         boom_verAngle = math.radians(float(input("Enter boom vertical angle (degrees): ")))
    #                         hook_height = float(input("Enter hook height: "))
    #                         crane_swarm.update_crane_state(crane_id, boom_horAngle=boom_horAngle, boom_verAngle=boom_verAngle, hook_height=hook_height)
                
    #     except ValueError:
    #         print("Invalid input. Please try again.")
    #     except KeyboardInterrupt:
    #         # Handle Ctrl+C explicitly
    #         signal_handler(None, None)

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
