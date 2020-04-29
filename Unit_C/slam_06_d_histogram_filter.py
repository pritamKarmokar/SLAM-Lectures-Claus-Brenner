# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)



# --->>> Copy your convolve(a, b) and multiply(a, b) functions here.
def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""

    dist_list = []

    for i in range(len(b.values)):
        delta = b.offset + i
        dist = move(a, delta)
        dist_list.append(dist)

    conv_result = Distribution.sum(dist_list, b.values)

    
    return conv_result



def multiply(a, b):

     """Multiply two distributions and return the resulting distribution."""

     # --->>> Put your code here.
     # min offset is the offset of the dist that starts latest
     min_offset = max(a.offset, b.offset)

     # max offset denotes offset of the dist that ends earliest
     max_offset = min(a.offset + len(a.values)-1, b.offset + len(b.values)-1)

     values = []

     # mutliplication will happen only between the min and max offset values computed above
     for o in range(min_offset, max_offset+1):
          a_offset = o - a.offset
          b_offset = o - b.offset
          value = a.values[a_offset] * b.values[b_offset]
          values.append(value)
     
     dist = Distribution(min_offset, values)
     dist.normalize()

    
     return dist  # Modify this to return your result.


if __name__ == '__main__':
    arena = (0,2200)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 100

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in range(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
