# The full Kalman filter, consisting of prediction and correction step.
#
# slam_07_f_kalman_filter
# Claus Brenner, 12.12.2012
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *
from slam_d_library import get_observations, write_cylinders


class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):

        # --->>> Put your method from 07_d_kalman_predict here.
        theta = state[2]
        l, r = control
        
        # compute alpha and R
        alpha = (r-l) / w

        if r != l:

            R = l / alpha
            # compute required elements of the G matrix
            G12 = (R + w/2)*(cos(theta + alpha) - cos(theta))
            G22 = (R + w/2)*(sin(theta + alpha) - sin(theta))

        else:

            # This is for the special case r == l.
            # compute required elements of the G matrix
            G12 = -l * sin(theta)
            G22 = l * cos(theta)

        m = array([[1, 0, G12], [0, 1, G22], [0, 0, 1]])  

        return m

    @staticmethod
    def dg_dcontrol(state, control, w):

        # --->>> Put your method from 07_d_kalman_predict here.
        theta = state[2]
        l, r = tuple(control)
        alpha = (r-l) / w

        if r != l:

            # This is for the case l != r.
            # Note g has 3 components and control has 2, so the result
            # will be a 3x2 (rows x columns) matrix.
            V11 = ((w*r) / ((r-l)**2)) * (sin(theta+alpha) - sin(theta))  - ((r+l) / (2*(r-l))) * cos(theta+alpha)
            V21 = ((w*r) / ((r-l)**2)) * (-cos(theta+alpha) + cos(theta)) - ((r+l) / (2*(r-l))) * sin(theta+alpha)
            
            V12 = -((w*l) / ((r-l)**2)) * (sin(theta+alpha) - sin(theta))  + ((r+l) / (2*(r-l))) * cos(theta+alpha)
            V22 = -((w*l) / ((r-l)**2)) * (-cos(theta+alpha) + cos(theta)) + ((r+l) / (2*(r-l))) * sin(theta+alpha)
            
        else:

            # This is for the special case l == r.
            V11 = (1/2) * (cos(theta) + (l/w) * sin(theta))            
            V21 = (1/2) * (sin(theta) - (l/w) * cos(theta)) 

            V12 = (1/2) * (cos(theta) - (l/w) * sin(theta))            
            V22 = (1/2) * (sin(theta) + (l/w) * cos(theta)) 


        # dg3_dl and dg3_dr are same for both cases
        V31 = -1 / w
        V32 = 1 / w

        m = array([[V11, V12], [V21, V22], [V31, V32]])  
            
        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):

        # --->>> Put your method from 07_d_kalman_predict here.
        left, right = control

        # compute control variance matrix
        sigma_l = (self.control_motion_factor * left)**2 + (self.control_turn_factor * (left-right))**2
        sigma_r = (self.control_motion_factor * right)**2 + (self.control_turn_factor * (left-right))**2
        control_covariance = diag([sigma_l, sigma_r])
        
        # compute G and V
        G = self.dg_dstate(self.state, control, self.robot_width)
        V = self.dg_dcontrol(self.state, control, self.robot_width)

        # computing new covariance matrix
        self.covariance = dot(dot(G, self.covariance), G.T) + dot(dot(V, control_covariance), V.T)

        # state' = g(state, control)
        self.state = self.g(self.state, control, self.robot_width)


    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):

        # --->>> Put your method from 07_e_measurement derivative here.
        # compute some variables for convenience
        x, y, theta = state        
        d = scanner_displacement
        x_m, y_m = landmark

        x_l = x + d * cos(theta)
        y_l = y + d * sin(theta)
        delta_x = x_m - x_l
        delta_y = y_m - y_l
        q = (x_m - x_l)**2 + (y_m - y_l)**2

        # compute H
        H11 = -delta_x / sqrt(q)
        H12 = -delta_y / sqrt(q)
        H13 = (d / sqrt(q)) * (delta_x * sin(theta) - delta_y * cos(theta))

        H21 = delta_y / q
        H22 = -delta_x / q
        H23 = (-d / q) * (delta_x * cos(theta) + delta_y * sin(theta)) - 1
 
        return array([[H11, H12, H13], [H21, H22, H23]]) 


    def correct(self, measurement, landmark):
        """The correction step of the Kalman filter."""

        # --->>> Put your new code here.
        #
        # You will have to compute:
        # H, using dh_dstate(...).
        # Q, a diagonal matrix, from self.measurement_distance_stddev and
        #  self.measurement_angle_stddev (remember: Q contains variances).
        # K, from self.covariance, H, and Q.
        #  Use linalg.inv(...) to compute the inverse of a matrix.
        # The innovation: it is easy to make an error here, because the
        #  predicted measurement and the actual measurement of theta may have
        #  an offset of +/- 2 pi. So here is a suggestion:
        #   innovation = array(measurement) -\
        #                self.h(self.state, landmark, self.scanner_displacement)
        #   innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        # Then, you'll have to compute the new self.state.
        # And finally, compute the new self.covariance. Use eye(3) to get a 3x3
        #  identity matrix.
        #
        # Hints:
        # dot(A, B) is the 'normal' matrix product (do not use: A*B).
        # A.T is the transposed of a matrix A (A itself is not modified).
        # linalg.inv(A) returns the inverse of A (A itself is not modified).
        # eye(3) returns a 3x3 identity matrix.

        # compute H and Q
        H = self.dh_dstate(self.state, landmark, self.scanner_displacement)
        Q = diag([self.measurement_distance_stddev**2, self.measurement_angle_stddev**2])

        # compute Kalman Gain
        K = dot(dot(self.covariance, H.T), linalg.inv(dot(dot(H, self.covariance), H.T) + Q))

        # compute innovation (as suggested)
        innovation = array(measurement) - self.h(self.state, landmark, self.scanner_displacement)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi

        # update state and covariance
        self.state = self.state + dot(K, innovation)
        self.covariance = dot((eye(3) - dot(K, H)), self.covariance)



if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0
    max_cylinder_distance = 300.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Measured start position.
    initial_state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width, scanner_displacement,
                              control_motion_factor, control_turn_factor,
                              measurement_distance_stddev,
                              measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the Kalman filter loop, with prediction and correction.
    states = []
    covariances = []
    matched_ref_cylinders = []
    for i in range(len(logfile.motor_ticks)):
        # Prediction.
        control = array(logfile.motor_ticks[i]) * ticks_to_mm
        kf.predict(control)

        # Correction.
        observations = get_observations(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset,
            kf.state, scanner_displacement,
            reference_cylinders, max_cylinder_distance)
        for j in range(len(observations)):
            kf.correct(*observations[j])

        # Log state, covariance, and matched cylinders for later output.
        states.append(kf.state)
        covariances.append(kf.covariance)
        matched_ref_cylinders.append([m[1] for m in observations])

    # Write all states, all state covariances, and matched cylinders to file.
    f = open("kalman_prediction_and_correction.txt", "w")
    for i in range(len(states)):
        # Output the center of the scanner, not the center of the robot.
        t = tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                            scanner_displacement * sin(states[i][2]),
                            0.0])
        f.write(f"F {t[0]} {t[1]} {t[2]}\n")
        # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
        e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
        t = (e + (sqrt(covariances[i][2,2]),))
        f.write(f"E {t[0]} {t[1]} {t[2]} {t[3]}\n")
        # Also, write matched cylinders.
        write_cylinders(f, "W C", matched_ref_cylinders[i])        

    f.close()
