from __future__ import annotations
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Twist


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(z: float, w: float) -> float:
    return wrap_angle(2.0 * math.atan2(z, w))


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class DockLOSPlanner(Node):
    def __init__(self):
        super().__init__("los_planner")

        
        self.declare_parameter("control_hz", 10.0)

        self.declare_parameter("safe_width", 0.8)
        self.declare_parameter("side_close_z", 3.0)
        self.declare_parameter("entrance_far_z", 4.0)
        self.declare_parameter("entrance_near_z", 3.0)
        self.declare_parameter("stop_z", 1.2)

        
        self.declare_parameter("lookahead", 2.5)
        self.declare_parameter("k_yaw_p", 0.2)   #0.08, 0.03, 0.01
        self.declare_parameter("k_yaw_i", 0.01)
        self.declare_parameter("k_yaw_d", 0.001)
        #self.declare_parameter("k_x", 0.1)

        self.declare_parameter("u_far", 0.1)
        self.declare_parameter("u_near", 0.05)
        self.declare_parameter("r_max", 0.3)

        
        self.declare_parameter("x_tol", 0.12)
        self.declare_parameter("yaw_tol", 0.12)

        
        self.declare_parameter("stern_docking", True)
        self.declare_parameter("search_spin_rate", 0.12)
        
        self.declare_parameter("delay_response", 0.5)
        self.delay_response = float(self.get_parameter("delay_response").value)

        self.control_hz = float(self.get_parameter("control_hz").value)

        self.safe_width = float(self.get_parameter("safe_width").value)
        self.side_close_z = float(self.get_parameter("side_close_z").value)
        self.entrance_far_z = float(self.get_parameter("entrance_far_z").value)
        self.entrance_near_z = float(self.get_parameter("entrance_near_z").value)
        self.stop_z = float(self.get_parameter("stop_z").value)

        self.lookahead = float(self.get_parameter("lookahead").value)

        self.k_yaw_p = float(self.get_parameter("k_yaw_p").value)
        self.k_yaw_i = float(self.get_parameter("k_yaw_i").value)
        self.k_yaw_d = float(self.get_parameter("k_yaw_d").value)
        #self.k_x = float(self.get_parameter("k_x").value)

        self.u_far = float(self.get_parameter("u_far").value)
        self.u_near = float(self.get_parameter("u_near").value)
        self.r_max = float(self.get_parameter("r_max").value)

        self.x_tol = float(self.get_parameter("x_tol").value)
        self.yaw_tol = float(self.get_parameter("yaw_tol").value)

        self.stern_docking = bool(self.get_parameter("stern_docking").value)
        self.search_spin_rate = float(self.get_parameter("search_spin_rate").value)

        
        self.pose_valid = False
        self.x = 0.0
        self.z = 999.0
        self.yaw = 0.0
        
        self.delay_start_time = None

        self.state = "SEARCH"
        
        self.linear_prev_x = 0.0
        self.angular_prev_z = 0.0
        self.linear_max_thrust_rate = 0.1
        self.angular_max_thrust_rate = 0.02
        
        self.docking_complete = False
        
        self.last_known_x    = 0.0
        self.last_known_z    = 999.0
        self.last_known_yaw  = 0.0
        self.tag_lost_time   = self.tag_lost_time = self.get_clock().now().nanoseconds / 1e9 
        self.TAG_LOST_TIMEOUT = 5.0   # time for retrieving the tag again before it again goes into search mode
        
        self.was_waiting = False
        
        self.yaw_error_integral = 0.0
        self.prev_yaw_error = 0.0
        self.prev_pid_time = self.get_clock().now().nanoseconds / 1e9

        
        
        self.sub_pose = self.create_subscription(
            PoseStamped,
            "/dock/filtered_pose",
            self.on_pose,
            10,
        )

        self.sub_valid = self.create_subscription(
            Bool,
            "/dock/filtered_pose_valid",
            self.on_valid,
            10,
        )

        self.pub_cmd = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.pub_state = self.create_publisher(
            String,
            "/dock/los_state",
            10,
        )

        self.pub_ref = self.create_publisher(
            PoseStamped,
            "/dock/los_reference",
            10,
        )

        self.timer = self.create_timer(1.0 / self.control_hz, self.step)

        self.get_logger().info("LOS planner/controller started!")

    # def on_valid(self, msg: Bool):
    #     valid = bool(msg.data)
    #     if valid and not self.pose_valid:
    #         self.delay_start_time= (self.get_clock().now().nanoseconds / 1e9)
            
    #     if not valid:
    #         self.delay_start_time = None
        
    #     self.pose_valid = valid
    def on_valid(self, msg: Bool):
        valid = bool(msg.data)
        if valid and not self.pose_valid:
            self.delay_start_time = self.get_clock().now().nanoseconds / 1e9
            self.tag_lost_time = None       # reset lost timer on recovery
        if not valid and self.pose_valid:
            self.tag_lost_time = self.get_clock().now().nanoseconds / 1e9
        self.pose_valid = valid
        

    # def on_pose(self, msg: PoseStamped):
    #     self.x = float(msg.pose.position.x)
    #     self.z = float(msg.pose.position.z)
    #     self.yaw = yaw_from_quat(
    #         float(msg.pose.orientation.z),
    #         float(msg.pose.orientation.w),
    #     )
    def on_pose(self, msg: PoseStamped):
        self.x   = float(msg.pose.position.x)
        self.z   = float(msg.pose.position.z)
        self.yaw = yaw_from_quat(
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        
        self.last_known_x   = self.x
        self.last_known_z   = self.z
        self.last_known_yaw = self.yaw

    def set_state(self, new_state: str):
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info(f"State -> {self.state}")
        self.pub_state.publish(String(data=self.state))

    def publish_reference(self, x_ref: float, z_ref: float, yaw_ref: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "dock_center"

        msg.pose.position.x = float(x_ref)
        msg.pose.position.y = 0.0
        msg.pose.position.z = float(z_ref)

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_ref / 2.0)
        msg.pose.orientation.w = math.cos(yaw_ref / 2.0)

        self.pub_ref.publish(msg)

    def speed_schedule(self) -> float:
        if self.z > self.entrance_near_z:
            return self.u_far
        return self.u_near

    def los_yaw_command(self, x_path: float = 0.0) -> float:
        e_ct = self.x - x_path
        
        dynamic_lookahead = clamp(
        self.lookahead * (self.z / self.entrance_far_z),
        1.0,             
        self.lookahead,  
        )
        
        
        yaw_des = math.atan2(-e_ct, dynamic_lookahead) #self.lookahead)
        return wrap_angle(yaw_des)

    def compute_cmd_from_los(self, x_path: float, allow_forward: bool) -> Twist:
        cmd = Twist()

        yaw_des = self.los_yaw_command(x_path=x_path)
        ############
        if not hasattr(self, 'prev_yaw_des'):
            self.prev_yaw_des = yaw_des

        alpha = 0.3   # 0 = very smooth, 1 = no filtering
        yaw_des = wrap_angle(
            self.prev_yaw_des + alpha * wrap_angle(yaw_des - self.prev_yaw_des)
        )
        self.prev_yaw_des = yaw_des
        ###############
        yaw_error = wrap_angle(yaw_des - self.yaw)
        
        if abs(yaw_error) < 0.03:
            yaw_error = 0.0

        ####### only P
        #r = self.k_yaw * yaw_error #- self.k_x * (self.x - x_path)
        #r = self.k_yaw * (yaw_error / math.pi) * self.r_max
        #r = clamp(r, -self.r_max, self.r_max)
        
        ############## PID
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.prev_pid_time

        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.control_hz

        self.prev_pid_time = now

        
        p_term = yaw_error

        self.yaw_error_integral += yaw_error * dt
        self.yaw_error_integral = clamp(self.yaw_error_integral, -1.0, 1.0)

        d_term = wrap_angle(yaw_error - self.prev_yaw_error) / dt
        self.prev_yaw_error = yaw_error

        r = (
            self.k_yaw_p * p_term
            + self.k_yaw_i * self.yaw_error_integral
            + self.k_yaw_d * d_term
        )

        r = clamp(r, -self.r_max, self.r_max)
        #########################        
        

        if allow_forward:
            u = self.speed_schedule()
            
            # if abs(self.x - x_path)>self.safe_width and self.z<self.side_close_z:
            #     u = 0.0
            if abs(yaw_error) > 0.9:
                u = self.u_near*0.6 #0.0
            elif abs(yaw_error)>0.5:
                u = self.u_near*0.8   #self.u_near * 0.4   #0.0
            elif abs(yaw_error)>0.2:
                u = self.u_near  #self.u_near * 0.7
             
        else:
            u = 0.0

        if self.stern_docking:
            cmd.linear.x = -abs(u)
        else:
            cmd.linear.x = abs(u)

        cmd.angular.z = -float(r)

        self.publish_reference(x_path, self.stop_z, yaw_des)

        return cmd

    def step(self):
        cmd = Twist()
        
        #x_abs = abs(self.x)
        # yaw_abs = abs(self.yaw)
        
        #x_path = math.copysign(self.safe_width, self.x)
        
        now = self.get_clock().now().nanoseconds / 1e9
        
        
        # SEARCH
        # if not self.pose_valid:
        #     self.set_state("SEARCH")
        #     cmd.linear.x = 0.0
        #     cmd.angular.z = self.search_spin_rate
        #     #self.pub_cmd.publish(cmd)
        #     #return
        if not self.pose_valid:
            now = self.get_clock().now().nanoseconds / 1e9
            #time_lost = (now - self.tag_lost_time) if self.tag_lost_time else 0.0
            if self.tag_lost_time is None:
                time_lost = self.TAG_LOST_TIMEOUT + 1.0 
            else:
                time_lost = now - self.tag_lost_time

            if time_lost < self.TAG_LOST_TIMEOUT:
                self.set_state("RECOVERING")

                recovery_r = clamp(
                    -self.last_known_x * 0.1,   
                    -self.search_spin_rate,
                    self.search_spin_rate,
                )
                cmd.linear.x  = 0.0
                cmd.angular.z = recovery_r
            else:
                self.set_state("SEARCH")
                cmd.linear.x  = 0.0
                cmd.angular.z = self.search_spin_rate
        
        
        #now = self.get_clock().now().nanoseconds / 1e9
        
        
        elif self.delay_start_time is not None and (now - self.delay_start_time) < self.delay_response:
            self.set_state("WAIT_FOR_STABLE_TAG")
            self.was_waiting = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        # elif self.delay_start_time is None:
        #     self.delay_start_time = now
            
        #     time_difference = now - self.delay_start_time
                
        #     if time_difference < self.delay_response:
        #         self.set_state("wait_for_stable_tag")
        #         cmd.linear.x = 0.0
        #         cmd.angular.z = 0.0
            
                #self.pub_cmd.publish(cmd)
                #return

        # x_abs = abs(self.x)
        # yaw_abs = abs(self.yaw)
        
        elif self.docking_complete:
            self.set_state("DOCKED")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.z <= self.stop_z:
            self.docking_complete = True
            self.set_state("DOCKED")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            #self.pub_cmd.publish(cmd)
            self.publish_reference(self.x, self.z, 0.0)
            #return

        ########################
        # elif x_abs > self.safe_width and self.z < self.side_close_z:
        #     self.set_state("REPOSITION")

        #     # x_path = math.copysign(self.safe_width, self.x)

        #     cmd = self.compute_cmd_from_los(
        #         x_path=x_path,
        #         allow_forward=True,
        #     )
        #############################    

            #self.pub_cmd.publish(cmd)
            #return

        elif self.z > self.entrance_far_z:
            self.set_state("APPROACH_ENTRANCE")

            cmd = self.compute_cmd_from_los(
                x_path=0.0,
                allow_forward=True,
            )

            #self.pub_cmd.publish(cmd)
            #return

        else:
            x_abs = abs(self.x)
            yaw_des_center = self.los_yaw_command(x_path=0.0)
            yaw_abs = abs(wrap_angle(yaw_des_center - self.yaw))
            
            dynamic_x_tol = clamp(
                self.x_tol * (self.z / self.entrance_near_z),
                0.08,   
                0.30,   
            )
            
            if x_abs > self.x_tol or yaw_abs > self.yaw_tol:
            #if x_abs > dynamic_x_tol or yaw_abs > self.yaw_tol:
                self.set_state("ALIGN_CENTERLINE")

                cmd = self.compute_cmd_from_los(
                    x_path=0.0,
                    allow_forward= True,  #False,
                )

                #self.pub_cmd.publish(cmd)
                #return

            
            else:
                self.set_state("FINAL_APPROACH")

                cmd = self.compute_cmd_from_los(
                    x_path=0.0,
                    allow_forward=True,
                )
                
        if self.was_waiting and self.state != "WAIT_FOR_STABLE_TAG":
            self.prev_linear_x  = 0.0
            self.prev_angular_z = 0.0
            self.was_waiting    = False
        
        cmd.linear.x = self.linear_prev_x + clamp(
            cmd.linear.x - self.linear_prev_x,
            -self.linear_max_thrust_rate,
            self.linear_max_thrust_rate,
        )
        
        cmd.angular.z = self.angular_prev_z + clamp(
            cmd.angular.z - self.angular_prev_z,
            -self.angular_max_thrust_rate,
            self.angular_max_thrust_rate,
        )
        
        self.linear_prev_x = cmd.linear.x
        self.angular_prev_z = cmd.angular.z
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DockLOSPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()