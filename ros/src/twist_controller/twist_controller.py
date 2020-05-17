
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
IS_DEBUG = True
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

class Controller(object):
    #def __init__(self, *args, **kwargs):
        # TODO: Implement
        #pass
    # init in dbw: 
    # self.controller = Controller(vehicle_mass = vehicle_mass, 
    #                                    fuel_capacity = fuel_capacity,
    #                                    brake_deadband = brake_deadband, 
    #                                    decel_limit = decel_limit,
    #                                    accel_limit = accel_limit,
    #                                    wheel_radius = wheel_radius, 
    #                                    wheel_base = wheel_base, 
    #                                    steer_ratio = steer_ratio,
    #                                    max_lat_accel = max_lat_accel, 
    #                                    max_steer_angle = max_steer_angle)

    # usage in dbw: 
    #self.throttle, self.brake, self.steering = self.controller.control(self.current_vel, 
    #                                                                            self.dbw_enabled, 
    #                                                                            self.linear_vel, 
    #                                                                            self.angular_vel)
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, 
                wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0
        mn = 0.     # mimnimum throttle value
        mx = 0.2    # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # velocity is noisy, so use low-pass filter
        tau = 0.5   # 1/ (2*pi*tau) = cutoff-frequence
        ts = 0.02   # sample time = 50 Hz
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()


    #def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #return 1., 0., 0.

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # get lowpass-filtered velocity
        current_vel = self.vel_lpf.filt(current_vel)

        #if IS_DEBUG:
        #    rospy.loginfo("Angular vel: {0}".format(angular_vel))
        #    rospy.loginfo("Target vel: {0}".format(linear_vel)
        #    rospy.loginfo("Target ang vel: {0}".format(angular_vel))
        #    rospy.loginfo("Current vel: {0}".format(current_vel))
        #    rospy.loginfo("Filtered vel: {0}".format(self.vel_lpf.get()))
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400 # Nm --> needed to hold car in place if we are at a light, resulting acceleration = -1 m/s^2
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)                        # limit brake value
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius      # torque = N * m = acceleration * mass * radius

        # problem with this controller: 
        # by the time, car is away from waypoint, waypoint_follower send new commands
        # 1. waypoint_folloer: make sure, it will update every time; if not following waypoints, do update
        # 2. change yaw-controller and add some damping terms (current_ang_vel vs target_ang_vel --> if too large...)        

        return throttle, brake, steering