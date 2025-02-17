import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import math

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
        self.id = id
        self.type = type
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
        self.hook_z = self.hook_height

    def move(self, new_x, new_y):
        """
        Move the crane to a new position.
        
        :param new_x: New X-coordinate.
        :param new_y: New Y-coordinate.
        """
        self.crane_x = new_x
        self.crane_y = new_y

    def check_collision(self, other_crane):
        """
        Check for collision with another crane.
        
        :param other_crane: Another TowerCrane object.
        :return: True if collision detected, False otherwise.
        """
        distance = np.sqrt((self.crane_x - other_crane.crane_x)**2 + (self.crane_y - other_crane.crane_y)**2)
        return distance < (self.boom_length + other_crane.boom_length)

    def __repr__(self):
        return f"TowerCrane(id={self.id}, crane_x={self.crane_x}, crane_y={self.crane_y}, boom_length={self.boom_length}, crane_z={self.crane_z})"

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
        self.id = id
        self.type = type
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
        self.boom_end_x = self.crane_x  + self.boom_length*math.cos(self.boom_horAngle)
        self.boom_end_y = self.crane_y  + self.boom_length*math.sin(self.boom_horAngle)
        self.boom_end_z = self.crane_z  + self.boom_length*math.sin(self.boom_verAngle)

        self.hook_x = self.crane_x  + self.boom_length*math.cos(self.boom_horAngle)
        self.hook_y = self.crane_y  + self.boom_length*math.sin(self.boom_horAngle)
        self.hook_z = self.hook_height


class CraneVisualizer:
    def __init__(self):
        """
        Initialize the visualizer.
        """
        self.cranes = []

    def add_crane(self, crane):
        """
        Add a crane to the visualizer.
        
        :param crane: TowerCrane object.
        """
        self.cranes.append(crane)

    def visualize_tower_crane_3D(self,crane,ax):
        # Plot the base of the crane
        ax.scatter(crane.crane_x, crane.crane_y, 0, color='blue')
        # Plot the tower of the crane
        ax.plot([crane.crane_x, crane.crane_x], [crane.crane_y, crane.crane_y], [0, crane.crane_z], color='red')
        # Plot the boom of the crane
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_y, crane.boom_end_y], [crane.crane_z, crane.boom_end_z], color='green')
        ax.text(crane.crane_x, crane.crane_y, crane.crane_z, f"Crane {crane.id}", fontsize=8)
        # Plot the trolley of the crane
        ax.scatter(crane.trolley_x, crane.trolley_y, crane.trolley_z, color='orange')
        # Plot the hook rope of the crane
        ax.plot([crane.trolley_x, crane.hook_x], [crane.trolley_y, crane.hook_y], [crane.trolley_z, crane.hook_z], color='purple',linestyle='--')
        # Plot the hook of the crane
        ax.scatter(crane.hook_x, crane.hook_y, crane.hook_z, color='purple')
    
    def visualize_tower_crane_2D(self,crane,ax):
        # Plot the base of the crane
        ax.scatter(crane.crane_x, 0, color='blue')
        # Plot the height of the crane (vertical line)
        ax.plot([crane.crane_x, crane.crane_x], [0, crane.crane_z], color='red')
        # Plot the boom (horizontal line at the top of the crane)
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_z, crane.boom_end_z], color='green')
        ax.text(crane.crane_x, crane.crane_z, f"Crane {crane.id}", fontsize=8)
        # Plot the trolley position on the boom
        ax.scatter(crane.trolley_x, crane.trolley_z, color='orange')
        # Plot the hook and rope (vertical line from the trolley to the hook height)
        ax.plot([crane.trolley_x, crane.hook_x], [crane.trolley_z, crane.hook_z], color='purple', linestyle='--')
        # Plot the hook 
        ax.scatter(crane.hook_x, crane.hook_z, color='purple')

    def visualize_luffing_jib_crane_3D(self,crane,ax):
        # Plot the base of the crane
        ax.scatter(crane.crane_x, crane.crane_y, 0, color='blue', label=f"Crane {crane.id}")
        # Plot the tower of the crane
        ax.plot([crane.crane_x, crane.crane_x], [crane.crane_y, crane.crane_y], [0, crane.crane_z], color='red')
        # Plot the boom of the crane
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_y, crane.boom_end_y], [crane.crane_z, crane.boom_end_z], color='green')
        # ax.text(crane.crane_x, crane.crane_y, crane.crane_z, f"Crane {crane.id}", fontsize=8)
        # Plot the hook rope of the crane
        ax.plot([crane.boom_end_x, crane.hook_x], [crane.boom_end_y, crane.hook_y], [crane.boom_end_z, crane.hook_z], color='purple',linestyle='--')
        # Plot the hook of the crane
        ax.scatter(crane.hook_x, crane.hook_y, crane.hook_z, color='purple')

    def visualize_luffing_jib_crane_2D(self,crane,ax):
        # Plot the base of the crane
        ax.scatter(crane.crane_x, 0, color='blue', label=f"Crane {crane.id}")
        # Plot the tower of the crane
        ax.plot([crane.crane_x, crane.crane_x], [0, crane.crane_z], color='red')
        # Plot the boom of the crane
        ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_z, crane.boom_end_z], color='green')
        ax.text(crane.crane_x, crane.crane_z, f"Crane {crane.id}", fontsize=8)
        # Plot the hook rope of the crane
        ax.plot([crane.boom_end_x, crane.hook_x], [crane.boom_end_z, crane.hook_z], color='purple',linestyle='--')
        # Plot the hook of the crane
        ax.scatter(crane.hook_x, crane.hook_z, color='purple')


    def visualize_3d(self,ax):
        """
        Visualize all cranes in a 3D plot with booms.
        """
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')

        for crane in self.cranes:
            if crane.type == "towerCrane":
                self.visualize_tower_crane_3D(crane,ax)
            elif crane.type == "luffingJibCrane":
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
        # fig, ax = plt.subplots()

        for crane in self.cranes:
            if crane.type == "towerCrane":
                self.visualize_tower_crane_2D(crane,ax)
            elif crane.type == "luffingJibCrane":
                self.visualize_luffing_jib_crane_2D(crane,ax)

        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Height (m)")
        ax.set_title("Front View (2D)")


    def visualize_top_view(self,ax):
        """
        Visualize all cranes in a 2D plot.
        """
        # plt.figure()
        # fig, ax = plt.subplots()
        for crane in self.cranes:
            ax.scatter(crane.crane_x, crane.crane_y, color='blue')
            ax.plot([crane.crane_x, crane.boom_end_x], [crane.crane_y, crane.boom_end_y], color='green')
            circle = plt.Circle((crane.crane_x, crane.crane_y), crane.boom_length, color='red', linestyle='--',fill=False)
            plt.gca().add_patch(circle)
            plt.text(crane.crane_x, crane.crane_y, f"Crane {crane.id}", fontsize=8, ha='center')
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title("Top View (2D)")

    
    def visualize_all(self):
        """
        Visualize all three views (3D, front view, top view) in the same figure.
        """
        fig = plt.figure(figsize=(18, 6))

        # 3D View
        ax1 = fig.add_subplot(131, projection='3d')
        self.visualize_3d(ax1)

        # Front View (2D)
        ax2 = fig.add_subplot(132)
        self.visualize_front_view(ax2)

        # Top View (2D)
        ax3 = fig.add_subplot(133)
        self.visualize_top_view(ax3)

        plt.tight_layout()
        plt.show()

    def check_all_collisions(self):
        """
        Check for collisions between all cranes.
        """
        for i in range(len(self.cranes)):
            for j in range(i + 1, len(self.cranes)):
                if self.cranes[i].check_collision(self.cranes[j]):
                    print(f"Collision detected between Crane {self.cranes[i].id} and Crane {self.cranes[j].id}")

def main():
    # Create cranes
    crane1 = TowerCrane(id=1, type = 'towerCrane', crane_x=20, crane_y=30, crane_z=50,  boom_length=15, boom_angle= math.radians(30), trolley_radius= 5, hook_height =5)
    crane2 = TowerCrane(id=2, type = 'towerCrane', crane_x=40, crane_y=50, crane_z=40,  boom_length=10, boom_angle= math.radians(60), trolley_radius= 5, hook_height =5)
    crane3 = LuffingJibCrane(id=3, type = 'luffingJibCrane', crane_x=60, crane_y=20, crane_z=20,  boom_length=20, boom_horAngle= math.radians(30), boom_verAngle= math.radians(60), hook_height =15)

    # Add cranes to visualizer
    visualizer = CraneVisualizer()
    visualizer.add_crane(crane1)
    visualizer.add_crane(crane2)
    visualizer.add_crane(crane3)

    visualizer.visualize_all()

    # Example of updating crane state in real time
    # while True:
    #     try:
    #         crane_id = int(input("Enter crane ID to update (1, 2, or 3): "))
    #         if crane_id == 1 or crane_id == 2:
    #             boom_angle = math.radians(float(input("Enter boom angle (degrees): ")))
    #             trolley_radius = float(input("Enter trolley radius: "))
    #             hook_height = float(input("Enter hook height: "))
    #             visualizer.update_crane_state(crane_id, boom_angle=boom_angle, trolley_radius=trolley_radius, hook_height=hook_height)
    #         elif crane_id == 3:
    #             boom_horAngle = math.radians(float(input("Enter boom horizontal angle (degrees): ")))
    #             boom_verAngle = math.radians(float(input("Enter boom vertical angle (degrees): ")))
    #             hook_height = float(input("Enter hook height: "))
    #             visualizer.update_crane_state(crane_id, boom_horAngle=boom_horAngle, boom_verAngle=boom_verAngle, hook_height=hook_height)
    #     except ValueError:
    #         print("Invalid input. Please try again.")


if __name__ == '__main__':
    main()
