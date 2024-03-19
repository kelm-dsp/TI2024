import numpy as np
import pylab as plt

filename = 'channel1.dat'
file = open(filename,"rb")
V=np.fromfile(filename,dtype='uint8')
nbvoie = 5
v0 = V[0::nbvoie]
v1 = V[1::nbvoie]
v2 = V[2::nbvoie]
v3 = V[3::nbvoie]
v4 = V[4::nbvoie]
plt.plot(v0)
plt.plot(v1)
plt.plot(v2)
plt.plot(v3)
plt.plot(v4)

plt.show()