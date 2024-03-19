import numpy as np
import serial

# Get handle to serial port
# (your port string may vary; windows users need 'COMn')
s = serial.Serial('COM9')
# open data file
nomfic="channel1.dat"
fd = open(nomfic, 'wb')
while True:
    # read one full chunk from the serial port
    data = s.read(50)
    # convert data to 8bit int numpy array
    data = np.frombuffer(data, dtype=np.uint8)
    # Convert array to float and rescale to voltage.
    # Assume 3.3V / 12bits
    # (we need calibration data to do a better job on this)
    # data = data.astype(np.float32) * (3.3 / 2**8)
    fd.write(data)
fd.close()

# import pylab as plt
#
# filename = 'channel1.dat'
# file = open(filename,"rb")
# V=np.fromfile(filename,dtype='uint16')
# nbvoie = 8
# v0 = V[0::nbvoie]
# v1 = V[1::nbvoie]
# v2 = V[2::nbvoie]
# v3 = V[3::nbvoie]
# v4 = V[4::nbvoie]
# v5 = V[5::nbvoie]
# v6 = V[6::nbvoie]
# v7 = V[7::nbvoie]
# plt.plot(v0)
# plt.plot(v1)
# plt.plot(v2)
# plt.plot(v3)
# plt.plot(v4)
# plt.plot(v5)
# plt.plot(v6)
# plt.plot(v7)
# plt.show()