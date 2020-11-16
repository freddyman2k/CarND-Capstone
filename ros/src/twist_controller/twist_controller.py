from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        
        self.yaw_controller = YawController(wheel_base=wheel_base, steer_ratio=steer_ratio, 
                                            min_speed=0.1, max_lat_accel=max_lat_accel, 
                                            max_steer_angle=max_steer_angle)
        
        min_throttle = 0
        max_throttle = 0.2
        self.throttle_controller = PID(kp=0.3, ki=0.1, kd=0, mn=min_throttle, mx=max_throttle)

        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = .02 # sample time
        self.vel_lowpass = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if not dbw_enabled: 
            # Drive-By-Wire is disabled in the car => Manual Drive
            # Reset the PID Controller to prevent the error from falsely accumulating
            self.throttle_controller.reset()
            return 0, 0, 0

        current_vel = self.vel_lowpass.filt(current_vel)

        steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # Cases where the car should brake:
        if linear_vel == 0 and current_vel < 0.1:
            # hold the car in place if we are stopped at a light
            throttle = 0
            brake = 700 # N*m, torque to prevent Carla from moving
        elif throttle < 0.1 and vel_error < 0:
            # car is going faster than the target velocity
            # and PID has already decreased the throttle
            # => brake
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m

        return throttle, brake, steer
