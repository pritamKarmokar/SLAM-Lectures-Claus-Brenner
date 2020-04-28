# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
import matplotlib.pyplot as plt # plot, show, ylim
from  matplotlib.lines import Line2D as  L2D
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    # how its called in our problem-> position = convolve(position, move_distribution)
    # --->>> Put your code here.
    dist_list = []

    # for every value in distribution 'b', we compute the corresponding offset i.e., delta
    # then move distribution 'a' by delta to get dist
    # then append dist to a list dist_list
    for i in range(len(b.values)):
        delta = b.offset + i
        dist = move(a, delta)
        dist_list.append(dist)

    # then use the sum function from Distribution class in distribution module to sum these up
    # summing should be weighted according to values in distribution 'b'
    conv_result = Distribution.sum(dist_list, b.values)

    
    return conv_result


if __name__ == '__main__':
    arena = (0,1000)

    # Move 3 times by 20.
    moves = [20] * 50

    # Start with a known position: probability 1.0 at position 10. i.e., Distribution(10, [1.0])
    position = Distribution.unit_pulse(10)
    plt.plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1], drawstyle='steps')

    # Now move and plot.
    for m in moves:
        # triangle (m, 2) is centered at m and has a half-width of +/-2
        # for this problem its always triangle(20, 2) i.e., Distribution(19, [0.25, 0.5, 0.25])
        move_distribution = Distribution.triangle(m, 2)
        position = convolve(position, move_distribution)
        # print(f'm: {m}, dist: {position}')
        plt.plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1], drawstyle='steps')


    # plt.ylim(0.0, 1.1)
    plt.show()
