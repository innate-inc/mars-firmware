import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
xs, ys = [], []
recording = False

while True:
    line = ser.readline().decode(errors='ignore').strip()
    if not line: 
        continue
    if line == '---BEGIN DATA---':
        recording = True
        continue
    if line == '---END DATA---':
        break
    if recording and ',' in line:
        try:
            x, y = map(int, line.split(','))
            xs.append(x); ys.append(y)
        except ValueError:
            pass

ser.close()

plt.figure()
plt.scatter(xs, ys, s=5)
plt.gca().set_aspect('equal', 'box')
plt.title('Magnetometer X vs Y')
plt.xlabel('magX'); plt.ylabel('magY')
plt.show()
