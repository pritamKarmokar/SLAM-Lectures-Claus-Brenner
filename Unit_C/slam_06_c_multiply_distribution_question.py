# Multiply a distribution by another distribution.
# 06_c_multiply_distribution
# Claus Brenner, 26 NOV 2012

from pylab import plot, show
from distribution import *


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
    arena = (0,1000)

    # Here is our assumed position. Plotted in blue.
    position_value = 400
    position_error = 100
    position = Distribution.triangle(position_value, position_error)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         color='b', drawstyle='steps')

    # Here is our measurement. Plotted in green.
    # That is what we read from the instrument.
    measured_value = 550
    measurement_error = 200
    measurement = Distribution.triangle(measured_value, measurement_error)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='g', drawstyle='steps')

    # Now, we integrate our sensor measurement. Result is plotted in red.
    position_after_measurement = multiply(position, measurement)
    plot(position_after_measurement.plotlists(*arena)[0],
         position_after_measurement.plotlists(*arena)[1],
         color='r', drawstyle='steps')

    show()
