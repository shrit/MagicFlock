from numpy import genfromtxt
import matplotlib.pyplot as plt
import numpy as np


num_lines = sum(1 for line in open('count16:18:15'))
print (num_lines)

y = np.loadtxt("count16:18:15")
x = np.arange(y.size)

z = np.mean(y)
print ("count deviation: ", np.std(y) )
print(z)

plt.plot(x, y, color='blue')

plt.xlabel('Number of flights')
plt.ylabel('Controller Counts before deformation')

#plt.title("Plot the deformation error according to the number of action ")

plt.savefig("count_high_res.esp", format='eps', dpi=1000)
plt.show()


