import numpy as np
import os
import csv
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

class DataProcessing():
    def __init__(self, data_directory="./DATA", filename="simulation_log.csv"):

        #robot1        
        self.accumulated_poses = np.zeros((1, 3), dtype=np.float64)
        self.accumulated_vels = np.zeros((1, 2), dtype=np.float64)

        #robot2
        self.accumulated_target_poses = np.zeros((1, 3), dtype=np.float64)
        self.accumulated_target_vels = np.zeros((1, 2), dtype=np.float64)
        
        #error(state)
        self.accumulated_error = np.zeros((1, 3), dtype=np.float64)

        self.accumulated_time = np.zeros((1, 1), dtype=np.float64)

        self.data_directory = data_directory
        self.filename = filename

        os.makedirs(data_directory, exist_ok=True)
        self.data_file_path = os.path.join(self.data_directory, self.filename)


    def store_data(self, pose, vel, target_pose, target_vel, error, elapsed_time):
        # Convert inputs into row vectors
        pose_array = np.array(pose).reshape(1, -1)
        vel_array = np.array(vel).reshape(1, -1)

        target_pose_array = np.array(target_pose).reshape(1, -1)
        target_vel_array = np.array(target_vel).reshape(1, -1)
    
        error_array = np.array(error).reshape(1, -1)

        elapsed_time_array = np.array(elapsed_time).reshape(1, -1)

        # Stack arrays properly
        self.accumulated_poses = np.vstack((self.accumulated_poses, pose_array))
        self.accumulated_vels = np.vstack((self.accumulated_vels, vel_array))

        self.accumulated_target_poses = np.vstack((self.accumulated_target_poses, target_pose_array))
        self.accumulated_target_vels = np.vstack((self.accumulated_target_vels, target_vel_array))
        
        self.accumulated_error = np.vstack((self.accumulated_error, error_array))

        self.accumulated_time = np.vstack((self.accumulated_time, elapsed_time_array))

        #self.save_to_csv()

        return
    
    def save_to_csv(self):

        # Combine all accumulated arrays horizontally
        data = np.hstack((
            self.accumulated_poses,
            self.accumulated_vels,
            self.accumulated_target_poses,
            self.accumulated_target_vels,
            self.accumulated_error,
            self.accumulated_time,
        ))
        
        # Define header names (adjust these to match your data structure)
        header = [
            'x', 'y', 'yaw',
            'vx', 'wz',
            'x_target', 'y_target', 'yaw_target',
            'vx_target', 'wz_target',
            'error_x', 'error_y', 'error_yaw',
            'time_step',
        ]
        
        #check if file exists
        file_exists = os.path.isfile(self.data_file_path)

        print(f"Writing data to: {self.data_file_path}")

        # Write data to CSV using the csv module
        with open(self.data_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(header)  # Write header row
            # Write each row of data; converting each row from a numpy array to a list
            for row in data:
                writer.writerow(row.tolist())


        print(f"Data stored in: {self.data_file_path}")

    def read_from_csv(self, file_path=None):
        if file_path is None:
            file_path = self.data_file_path
        
        try:
            if not os.path.isfile(file_path):
                print(f"Error: CSV file not found at {file_path}")
                return None
            
            print(f"Reading data from: {file_path}")
            
            # Read the CSV file
            data = []
            with open(file_path, mode='r', newline='') as file:
                reader = csv.reader(file)
                header = next(reader)  # Skip header row
                for row in reader:
                    # Convert each value to float
                    data.append([float(val) for val in row])
            
            # Convert to numpy array
            data = np.array(data)
            
            if data.size == 0:
                print("Error: CSV file is empty or has no data rows")
                return None
            
            # Parse data based on column structure defined in header
            self.accumulated_poses = data[:, 0:3]
            self.accumulated_vels = data[:, 3:5]
            self.accumulated_target_poses = data[:, 5:8]
            self.accumulated_target_vels = data[:, 8:10]
            self.accumulated_error = data[:, 10:13]
            self.accumulated_time = data[:, 13:14]
            
            print(f"Successfully loaded data from {file_path}")
            print(f"Loaded {len(data)} data points")
            return data
            
        except Exception as e:
            print(f"Error reading CSV file: {str(e)}")
            return None
        
    def plot_2d(self, accumulated_poses, accumulated_target_poses):        
        
        # Handle the specific data structure we're getting
        if isinstance(accumulated_poses, tuple) and len(accumulated_poses) == 1:
            # This is a tuple containing a single numpy array
            pose_array = accumulated_poses[0]  # Extract the numpy array from the tuple
            follower_x = pose_array[:, 0]
            follower_y = pose_array[:, 1]
            follower_yaw = pose_array[:, 2]
        else:
            # Handle other cases (direct numpy array)
            follower_x = accumulated_poses[:, 0]
            follower_y = accumulated_poses[:, 1]
            follower_yaw = accumulated_poses[:, 2]
        
        if isinstance(accumulated_target_poses, tuple) and len(accumulated_target_poses) == 1:
            # This is a tuple containing a single numpy array
            target_array = accumulated_target_poses[0]  # Extract the numpy array from the tuple
            target_x = target_array[:, 0]
            target_y = target_array[:, 1]
            target_yaw = target_array[:, 2]
        else:
            # Handle other cases
            target_x = accumulated_target_poses[:, 0]
            target_y = accumulated_target_poses[:, 1]
            target_yaw = accumulated_target_poses[:, 2]
        
        # Ensure data is in numpy array format
        follower_x = np.array(follower_x)
        follower_y = np.array(follower_y)
        follower_yaw = np.array(follower_yaw)
        target_x = np.array(target_x)
        target_y = np.array(target_y)
        target_yaw = np.array(target_yaw)
        
        # Create time steps for color gradient
        time_steps = np.arange(len(follower_x) - 1)
        
        # Create figure and axis
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Plot follower and target paths with color gradient
        sc_follower = ax.scatter(follower_x[1:], follower_y[1:], c=np.arange(len(follower_x)-1), cmap='viridis', 
                            marker='o', s=15, label='Follower Position')
        sc_target = ax.scatter(target_x[1:], target_y[1:], c=np.arange(len(target_x)-1), cmap='plasma', 
                            marker='^', s=25, label='Target Position')
        
        # Add colorbars
        cbar_ax1 = fig.add_axes([0.92, 0.6, 0.02, 0.25])  # [left, bottom, width, height]
        cbar_ax2 = fig.add_axes([0.92, 0.25, 0.02, 0.25])
        
        cbar_follower = plt.colorbar(sc_follower, cax=cbar_ax1)
        cbar_follower.set_label('Time Step (Follower)')
        cbar_target = plt.colorbar(sc_target, cax=cbar_ax2)
        cbar_target.set_label('Time Step (Target)')
        
        # Draw the overall paths as semi-transparent lines
        ax.plot(follower_x[1:], follower_y[1:], color='blue', alpha=0.3)  # Skip first point
        ax.plot(target_x[1:], target_y[1:], color='red', linestyle='--', alpha=0.3)  # Skip first point
        
        # Plot yaw arrows
        arrow_length = 0.2  # Determine arrow length based on data scale
        step = 7  # Plot every Nth arrow to avoid clutter
        
        # Follower yaw arrows
        for i in range(0, len(follower_x), step):
            dx = arrow_length * np.cos(follower_yaw[i])
            dy = arrow_length * np.sin(follower_yaw[i])
            
            if i == 0:
                # q_follower = ax.quiver(follower_x[i], follower_y[i], dx, dy, 
                #         color='red', scale=1, scale_units='xy', angles='xy')
                continue
            else:
                ax.quiver(follower_x[i], follower_y[i], dx, dy, 
                        color='red', scale=1, scale_units='xy', angles='xy')
        
        # Target yaw arrows
        for j in range(0, len(target_x), step):
            dx = arrow_length * np.cos(target_yaw[j])
            dy = arrow_length * np.sin(target_yaw[j])
            
            if j == 0:
                # q_target = ax.quiver(target_x[i], target_y[i], dx, dy, 
                #         color='black', scale=1, scale_units='xy', angles='xy')
                continue
            else:
                ax.quiver(target_x[j], target_y[j], dx, dy, 
                        color='black', scale=1, scale_units='xy', angles='xy')
        
        # Label axes and set plot title
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title("Robot Trajectories with Yaw Orientation")
        
        # Add legend for quiver arrows too
        handles, labels = ax.get_legend_handles_labels()
        handles.extend([Line2D([0], [0], marker='', color='red', linestyle='-', linewidth=2),
                    Line2D([0], [0], marker='', color='black', linestyle='-', linewidth=2)])
        labels.extend(['Follower Yaw', 'Target Yaw'])
        ax.legend(handles, labels)
        
        # Equal aspect ratio
        ax.set_aspect('equal')
        
        # Show the plot
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        return fig
    
    def plot_info(self, accumulated_vels, accumulated_target_vels, accumulated_error, accumulated_time):
        """
        Plot velocity and error information across time
        
        Parameters:
        accumulated_vels: Array with shape (n, 2) containing [vx, wz] for the follower robot
        accumulated_target_vels: Array with shape (n, 2) containing [vx, wz] for the target robot
        accumulated_error: Array with shape (n, 3) containing [x_e, y_e, theta_e] errors
        accumulated_time: Array with shape (n, 1) containing time stamps
        """
        import matplotlib.pyplot as plt
        import numpy as np
        
        # Handle tuple case similar to plot_2d
        if isinstance(accumulated_vels, tuple) and len(accumulated_vels) == 1:
            accumulated_vels = accumulated_vels[0]
        if isinstance(accumulated_target_vels, tuple) and len(accumulated_target_vels) == 1:
            accumulated_target_vels = accumulated_target_vels[0]
        if isinstance(accumulated_error, tuple) and len(accumulated_error) == 1:
            accumulated_error = accumulated_error[0]
        if isinstance(accumulated_time, tuple) and len(accumulated_time) == 1:
            accumulated_time = accumulated_time[0]
        
        # Skip the first element (initialized as zeros)
        v_follower = accumulated_vels[1:, 0]       # Linear velocity of follower
        w_follower = accumulated_vels[1:, 1]       # Angular velocity of follower
        
        v_target = accumulated_target_vels[1:, 0]  # Linear velocity of target
        w_target = accumulated_target_vels[1:, 1]  # Angular velocity of target
        
        error_x = accumulated_error[1:, 0]         # X error
        error_y = accumulated_error[1:, 1]         # Y error
        error_yaw = accumulated_error[1:, 2]       # Yaw error
        
        # Calculate xy error magnitude (Euclidean distance)
        xy_error = np.sqrt(error_x**2 + error_y**2)
        
        # Create time vector - use the actual time values or create from indices
        if accumulated_time.size > 1:
            time_steps = accumulated_time[1:, 0]  # Use actual time values
        else:
            time_steps = np.arange(len(v_follower))  # Create from indices
        
        # Create figure with 4 subplots (2x2 grid)
        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
        
        # Subplot 1: Linear Velocities Comparison
        axs[0, 0].plot(time_steps, v_follower, label="Follower Linear Vel", color="blue", linestyle="-")
        axs[0, 0].plot(time_steps, v_target, label="Target Linear Vel", color="red", linestyle="--")
        axs[0, 0].set_title("Linear Velocity Comparison")
        axs[0, 0].set_xlabel("Time (s)")
        axs[0, 0].set_ylabel("Velocity (m/s)")
        axs[0, 0].grid(True)
        axs[0, 0].legend()
        
        # Subplot 2: Angular Velocities Comparison
        axs[0, 1].plot(time_steps, w_follower, label="Follower Angular Vel", color="blue", linestyle="-")
        axs[0, 1].plot(time_steps, w_target, label="Target Angular Vel", color="red", linestyle="--")
        axs[0, 1].set_title("Angular Velocity Comparison")
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].set_ylabel("Angular Velocity (rad/s)")
        axs[0, 1].grid(True)
        axs[0, 1].legend()
        
        # Subplot 3: XY Error over time
        axs[1, 0].plot(time_steps, xy_error, label="XY Error", color="green")
        axs[1, 0].set_title("XY Position Error vs Time")
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].set_ylabel("Error (m)")
        axs[1, 0].grid(True)
        axs[1, 0].legend()
        
        # Subplot 4: Yaw Error over time
        axs[1, 1].plot(time_steps, error_yaw, label="Yaw Error", color="purple")
        axs[1, 1].set_title("Yaw Error vs Time")
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Error (rad)")
        axs[1, 1].grid(True)
        axs[1, 1].legend()
        
        # Add a common title for the entire figure
        fig.suptitle("Robot Control Performance Analysis", fontsize=16)
        
        # Adjust layout and show
        plt.tight_layout(rect=[0, 0, 1, 0.96])  # Make room for suptitle
        plt.show()
        
        # Final statistics
        print(f"Number of data points: {len(time_steps)}")
        print(f"Final XY Error: {xy_error[-1]:.4f} m")
        print(f"Final Yaw Error: {error_yaw[-1]:.4f} rad")
        
        return fig

    def plot_data(self, csv_path=None, save_fig=True):

        if csv_path is not None:
            print(f"No data stored in class object, reading from csv_path: {csv_path}")

            data = self.read_from_csv(csv_path)

            if data is not None:
                accumulated_poses = data[:, 0:3]
                accumulated_vels = data[:, 3:5]
                accumulated_target_poses = data[:, 5:8]
                accumulated_target_vels = data[:, 8:10]
                accumulated_error = data[:, 10:13]
                accumulated_time = data[:, 13:14]
        else:
            print("Using data stored in the class object.")
            accumulated_poses = self.accumulated_poses,
            accumulated_vels = self.accumulated_vels,
            accumulated_target_poses = self.accumulated_target_poses,
            accumulated_target_vels = self.accumulated_target_vels,
            accumulated_error = self.accumulated_error,
            accumulated_time = self.accumulated_time,

        fig1 = self.plot_2d(accumulated_poses, accumulated_target_poses)
        fig2 = self.plot_info(accumulated_vels, accumulated_target_vels, accumulated_error, accumulated_time)

        if save_fig is True:
            print(f"Proceeding saving the figures in directory: {self.data_directory}")
            fig1.savefig(os.path.join(self.data_directory, "trajectory_plot.png"), dpi=300, bbox_inches='tight')
            fig2.savefig(os.path.join(self.data_directory, "performance_metrics.png"), dpi=300, bbox_inches='tight')



# if __name__ == '__main__':
#     data = DataProcessing(data_directory = "./CSIAU_DATA")

#     data.plot_data(csv_path="/home/joao/Documents/csiau_ws/CSIAU_DATA/simulation_log.csv", save_fig = True)
