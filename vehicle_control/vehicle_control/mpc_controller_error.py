import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Path
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import asyncio
import time
import csv
import os
from vehicle_control.data_processing import DataProcessing

OUTPUT_DIRECTORY = "./CSIAU_DATA" 
FILENAME = "simulation_log.csv"
LOG_AND_PLOT = False
#launch do comando ps4: ros2 launch p9n_bringup teleop.launch.py   topic_name:=/robot2/cmd_vel linear_speed:=1.0 angular_speed:=1.0
#parar controlador: ros2 topic pub --once /stop_controller std_msgs/Bool "data: true"

class ErrorBasedMPCNode(Node):
    def __init__(self, follower_topic, target_topic):
        super().__init__('error_based_mpc_node') 

        # MPC parameters
        self.horizon = 100
        self.dt = 0.1
        self.elapsed_time = float(0.0)
        self.compute_time = float(0.0)

        # Error State: [x_e, y_e, theta_e] - error in robot frame
        # Control Input: [v_rob, w_rob] - robot linear and angular velocity
        self.num_states = 3
        self.num_control = 2

        # Weights for the cost function 
        self.Q = ca.diag([
            4.0,   # x_e error
            4.0,   # y_e error
            1.0,   # theta_e error
        ])

        self.Qf = ca.diag([
            8.0,   # x_e terminal error
            8.0,   # y_e terminal error
            2.0    # theta_e terminal error
        ])

        self.R = ca.diag([
            0.5,   # v_rob (linear velocity input)
            0.05,  # w_rob (angular velocity input)
        ])

        self.S = ca.diag([
            0.1,   # change in v_rob
            0.5,   # change in w_rob
        ])

        # Constraints
        self.error_x_constraint = [-100, 100]
        self.error_y_constraint = [-100, 100]
        self.error_theta_constraint = [-np.pi, np.pi]
        self.v_constraint = [-2, 2]
        self.w_constraint = [-4.0, 4.0]

        # Initialize error state
        self.error_state = ca.DM.zeros(self.num_states)  # [x_e, y_e, theta_e]
        self.current_follower_state = ca.DM.zeros(self.num_states)  # [x_rob, y_rob, theta_rob]
        self.current_goal_state = ca.DM.zeros(self.num_states)      # [x_ref, y_ref, theta_ref]
        
        # Control inputs
        self.current_input = ca.DM.zeros(2)   # [v_rob, w_rob]
        self.target_input = ca.DM.zeros(2)    # [v_ref, w_ref]

        # Setup optimization problem
        self.setup_mpc()
        self.controller_running = True

        # ROS2 publishers and subscribers
        self.follower_subscription = self.create_subscription(
            Odometry,
            follower_topic,
            self.follower_pose_cb,
            10
        )
        self.target_subscription = self.create_subscription(
            Odometry,
            target_topic,
            self.target_pose_cb,
            10
        )

        self.stop_subscription = self.create_subscription(
            Bool,
            '/stop_controller',
            self.stop_callback,
            10
        )

        self.velocity_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 100)
        self.control_msg = Twist()

        self.check_follower = False
        self.check_target = False

        # Timer for continuous control in seconds 10hz
        self.control_timer = self.create_timer(self.dt, self.control_callback)

        self.data_logger = DataProcessing(data_directory=OUTPUT_DIRECTORY, filename=FILENAME)

        self.get_logger().info('ErrorBasedMPCNode has been initialized')

    
    def extract_yaw(self, Q):
        q0 = Q[0]  # w
        q1 = Q[1]  # x
        q2 = Q[2]  # y
        q3 = Q[3]  # z

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array(yaw).reshape(1, -1)

    def stop_callback(self, msg):
        if msg.data:
            print("Stop command received! Stopping controller...")
            self.controller_running = False

    def follower_pose_cb(self, msg):
        # Get follower position (robot)
        self.current_follower_state[0:2] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])        
        
        quaternion = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        yaw = self.extract_yaw(quaternion)
        self.current_follower_state[2] = yaw
        
        # Get follower inputs
        self.current_input[0] = np.array([msg.twist.twist.linear.x])
        self.current_input[1] = np.array([msg.twist.twist.angular.z])

        self.check_follower = True
        
        # Update error state if target is also available
        if self.check_target:
            self.update_error_state()

    def target_pose_cb(self, msg):
        # Get target position
        target_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])
        
        # Get quaternion and extract yaw
        quaternion = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])
        yaw = float(self.extract_yaw(quaternion))
        
        # Calculate the position 1.5m behind the target vehicle
        behind_offset_x = -1.5 * np.cos(yaw)  # 1.5m in the negative heading direction
        behind_offset_y = -1.5 * np.sin(yaw)
        
        # Calculate goal position
        goal_position_x = target_position[0] + behind_offset_x
        goal_position_y = target_position[1] + behind_offset_y
        
        # Set the goal state
        self.current_goal_state[0] = float(goal_position_x)
        self.current_goal_state[1] = float(goal_position_y)
        self.current_goal_state[2] = yaw  # Same orientation as the target
        
        # Get target inputs
        self.target_input[0] = float(msg.twist.twist.linear.x)
        self.target_input[1] = float(msg.twist.twist.angular.z)
        
        self.check_target = True
        
        # Update error state if follower is also available
        if self.check_follower:
            self.update_error_state()

    def update_error_state(self):
        """
        Calculate the error state [x_e, y_e, theta_e] in the robot's frame
        using the transformation in equation (4) from the image.
        """
        # Extract states
        x_rob = float(self.current_follower_state[0])
        y_rob = float(self.current_follower_state[1])
        theta_rob = float(self.current_follower_state[2])
        
        x_ref = float(self.current_goal_state[0])
        y_ref = float(self.current_goal_state[1])
        theta_ref = float(self.current_goal_state[2])
        
        # Calculate global frame errors
        dx = x_ref - x_rob
        dy = y_ref - y_rob
        dtheta = theta_ref - theta_rob
        
        # Normalize angle difference to [-pi, pi]
        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
        
        # Transform to robot's local frame using the rotation matrix in eq. (4)
        cos_theta = np.cos(theta_rob)
        sin_theta = np.sin(theta_rob)
        
        x_e = cos_theta * dx + sin_theta * dy
        y_e = -sin_theta * dx + cos_theta * dy
        theta_e = dtheta
        
        # Update error state
        self.error_state[0] = x_e
        self.error_state[1] = y_e
        self.error_state[2] = theta_e
        

    def error_kinematics(self, input, error_state, ref_input):
        """
        Error dynamics model as defined in equation (5) from the image.
        
        Args:
            input: [v_rob, w_rob] - robot control inputs
            error_state: [x_e, y_e, theta_e] - error state in robot frame
            ref_input: [v_ref, w_ref] - reference (target) control inputs
            
        Returns:
            Derivatives of error states [dx_e/dt, dy_e/dt, dtheta_e/dt]
        """
        # Extract states and inputs
        x_e = error_state[0]
        y_e = error_state[1]
        theta_e = error_state[2]
        
        v_rob = input[0]  # Linear velocity of robot
        w_rob = input[1]  # Angular velocity of robot
        
        v_ref = ref_input[0]  # Linear velocity of reference/target
        w_ref = ref_input[1]  # Angular velocity of reference/target
        
        # Implement error dynamics from equation (5)
        dx_e = w_rob * y_e - v_rob + v_ref * ca.cos(theta_e)
        dy_e = -w_rob * x_e + v_ref * ca.sin(theta_e)
        dtheta_e = w_ref - w_rob
        
        # Return the error state derivatives
        return ca.vertcat(dx_e, dy_e, dtheta_e)

    def setup_mpc(self):
        # CasADi symbols
        # Error state: [x_e, y_e, theta_e]
        # Control Input: [v_rob, w_rob]
        self.x = ca.MX.sym('x', self.num_states)  # Error state
        self.u = ca.MX.sym('u', self.num_control)  # Robot control input
        self.u_ref = ca.MX.sym('u_ref', self.num_control)  # Reference (target) input
        
        # Error dynamics
        x_derivative = self.error_kinematics(input=self.u, error_state=self.x, ref_input=self.u_ref)
        
        # Euler integration of the state 
        x_next = self.x + self.dt * x_derivative
        
        # Define the state transition function
        self.f = ca.Function('f', [self.x, self.u, self.u_ref], [x_next])
        
        # Optimization variables
        self.opt_x = ca.MX.sym('opt_x', self.num_states, self.horizon + 1)  # Error states
        self.opt_u = ca.MX.sym('opt_u', self.num_control, self.horizon)     # Control inputs
        
        # Parameters - separated into initial error state and reference inputs
        self.p_error = ca.MX.sym('p_error', self.num_states)  # Initial error state
        self.p_ref = ca.MX.sym('p_ref', self.num_control, self.horizon)  # Reference inputs over horizon
        
        # Cost/Objective function
        obj = 0
        delta_u_cost = 0

        for k in range(self.horizon):
            # State cost - minimize error
            # The error is already in the state, so we want to drive it to zero
            state_error = self.opt_x[:, k]  # Error relative to zero
            state_cost = ca.mtimes([state_error.T, self.Q, state_error])
            
            # Input change cost
            if k > 0:  # For k=0, we don't have a previous input to compare
                du = self.opt_u[:, k] - self.opt_u[:, k-1]
                delta_u_cost = ca.mtimes([du.T, self.S, du])
            
            # Control input cost
            control_cost = ca.mtimes([self.opt_u[:, k].T, self.R, self.opt_u[:, k]])
            
            # Add costs to objective
            obj += state_cost + control_cost + delta_u_cost
        
        # Terminal cost - higher penalty on final error
        final_state_error = self.opt_x[:, self.horizon]  # Final error state
        final_state_cost = ca.mtimes([final_state_error.T, self.Qf, final_state_error])
        obj += final_state_cost
        
        # Define constraints
        g = []
        
        # Dynamics constraints
        for k in range(self.horizon):
            # Get reference input for this step directly from p_ref
            u_ref_k = self.p_ref[:, k]
            
            # Constrain next state to follow error dynamics
            g.append(self.opt_x[:, k+1] - self.f(self.opt_x[:, k], self.opt_u[:, k], u_ref_k))
        
        # Initial condition constraint
        g.append(self.opt_x[:, 0] - self.p_error)
        
        # NLP problem
        nlp = {
            'x': ca.vertcat(ca.reshape(self.opt_x, -1, 1), ca.reshape(self.opt_u, -1, 1)),
            'f': obj,
            'g': ca.vertcat(*g),
            'p': ca.vertcat(self.p_error, ca.reshape(self.p_ref, -1, 1))
        }
        
        # Create solver
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.constr_viol_tol': 1e-4, 'ipopt.max_iter': 5000}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
    
    async def control_callback(self):
        if not (self.check_follower and self.check_target):
            self.get_logger().warn("Waiting for initial follower/target states...")
            return
        
        # For debugging: print current error state
        print(f"Error state [x_e, y_e, theta_e]: {self.error_state}")

        if not self.controller_running:
            print("Stopping controller timer ...")
            self.control_timer.cancel()
            if LOG_AND_PLOT is True:
                self.data_logger.save_to_csv()
                self.data_logger.plot_data()
            return

        # Prepare reference inputs matrix for the horizon
        # (assuming constant reference input throughout the horizon)
        ref_inputs = np.tile(np.array(self.target_input).flatten(), (self.horizon, 1)).T
        
        start_solving = time.time()
        
        # Solve MPC problem
        x0 = np.zeros((self.num_states, self.horizon + 1))
        x0[:, 0] = np.array(self.error_state).flatten()  # Initial error state
        u0 = np.zeros((self.num_control, self.horizon))  # Initial guess for control
        
        res = self.solver(
            # Initial Guess
            x0=ca.vertcat(ca.reshape(x0, -1, 1), ca.reshape(u0, -1, 1)),
            # Lower bound for the states and inputs
            lbx=ca.vertcat(
                [self.error_x_constraint[0], self.error_y_constraint[0], self.error_theta_constraint[0]] * (self.horizon + 1), 
                [self.v_constraint[0], self.w_constraint[0]] * self.horizon
            ),
            # Upper bound for the states and inputs
            ubx=ca.vertcat(
                [self.error_x_constraint[1], self.error_y_constraint[1], self.error_theta_constraint[1]] * (self.horizon + 1), 
                [self.v_constraint[1], self.w_constraint[1]] * self.horizon
            ),
            # Constraint bounds (all equality constraints)
            lbg=0,  # g = 0
            ubg=0,  # g = 0
            # Parameters: initial error state and reference inputs over horizon
            p=ca.vertcat(self.error_state, ca.reshape(ca.DM(ref_inputs), -1, 1))
        )
        
        # Extract optimal control input
        u_opt = np.array(res['x'][-2*self.horizon:]).reshape(self.horizon, 2)
        
        # Apply first control input
        self.control_msg.linear.x = float(u_opt[0, 0])   # v_rob
        self.control_msg.angular.z = float(u_opt[0, 1])  # w_rob
        
        # Publish control input
        self.velocity_pub.publish(self.control_msg)
        
        # Update timing information
        end_solving = time.time()
        self.compute_time = (end_solving - start_solving)        
        
        #log data:
        self.data_logger.store_data(pose=self.current_follower_state, vel=self.current_input,
                                  target_pose=self.current_goal_state, target_vel=self.target_input,
                                  error=self.error_state,
                                  elapsed_time=self.elapsed_time)
        
        self.elapsed_time += self.dt
        self.elapsed_time = round(self.elapsed_time, 2)  # For precision


        
        self.get_logger().info(f'Elapsed Time: {self.elapsed_time} (s) \nOptimization time: {self.compute_time} (s)')

def main(args=None):
    rclpy.init(args=args)

    follower_topic = "/robot1/odom"
    target_topic = "/robot2/odom"

    node = ErrorBasedMPCNode(follower_topic, target_topic)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()