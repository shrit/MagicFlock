from numpy import genfromtxt
import matplotlib.pyplot as plt
import numpy as np


num_lines = sum(1 for line in open('error'))
print (num_lines)

y = np.loadtxt("error")
x = np.arange(y.size)

print(y)

z = np.mean(y)

print ("error standard deviation : ", np.std(y))

print(z)
#plt.ylim(-1, +9)

plt.plot(x, y, color='blue')


plt.xlabel('Number of flight')
plt.ylabel('Error of deformation combined totatly in m')


fig = plt.subplot()

#fig.savefig("test.png")
plt.show()


