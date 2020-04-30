 # The particle filter, prediciton only.
#
# slam_08_a_particle_prediciton.
# Claus Brenner, 04.01.2013
from lego_robot import *
from math import sin, cos, pi, sqrt
import random

class ParticleFilter:
    def __init__(self, initial_particles,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor):
        # The particles.
        self.particles = initial_particles

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor

    # State transition. This is exactly the same method as in the Kalman filter.
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

        return (g1, g2, g3)

    def predict(self, control):
        """The prediction step of the particle filter."""
        left, right = control

        # --->>> Put your code here.

        # Compute left and right variance.
        # alpha_1 is self.control_motion_factor.
        # alpha_2 is self.control_turn_factor.
        # Then, do a loop over all self.particles and construct a new
        # list of particles.
        # In the end, assign the new list of particles to self.particles.
        # For sampling, use random.gauss(mu, sigma). (Note sigma in this call
        # is the standard deviation, not the variance.)
        
        # compute left and right stddevs
        sigma_l = sqrt((self.control_motion_factor * left)**2 + (self.control_turn_factor * (left-right))**2)
        sigma_r = sqrt((self.control_motion_factor * right)**2 + (self.control_turn_factor * (left-right))**2)

        # loop over particles sample new left and right controls and new particles
        for i in range(len(self.particles)):
            # sample control for i-th particle
            l = random.gauss(left, sigma_l)
            r = random.gauss(right, sigma_r)
            new_sampled_control = (l,r)

            # use state transition function g, to update i-th particle
            self.particles[i] = self.g(self.particles[i], new_sampled_control, self.robot_width)




        

    def print_particles(self, file_desc):
        """Prints particles to given file_desc output."""
        if not self.particles:
            return
        file_desc.write(f"PA ")
        for p in self.particles:
            file_desc.write(f"{p[0]:.0f} {p[1]:.0f} {p[2]:.3f} ")
        file_desc.write('\n')


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 300
    measured_state = (1850.0, 1897.0, 213.0 / 180.0 * pi)
    standard_deviations = (100.0, 100.0, 10.0 / 180.0 * pi)
    initial_particles = []
    for i in range(number_of_particles):
        initial_particles.append(tuple([
            random.gauss(measured_state[j], standard_deviations[j])
            for j in range(3)]))

    # Setup filter.
    pf = ParticleFilter(initial_particles,
                        robot_width, scanner_displacement,
                        control_motion_factor, control_turn_factor)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records.
    # This is the particle filter loop, with prediction and correction.
    f = open("particle_filter_predicted.txt", "w")
    for i in range(len(logfile.motor_ticks)):
        # Prediction.
        control = list(map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i]))
        pf.predict(control)

        # Output particles.
        pf.print_particles(f)

    f.close()
