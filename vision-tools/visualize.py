import matplotlib.pyplot as plt
import os
import argparse
import numpy as np

def visualizeIMU(path):
    print("Visualizing IMU data from: ", path)

    imuFile = os.path.join(path, "day_IMU.txt")

    f = open(imuFile, 'r')
   
    T = []
    W = []
    A = []

    for line in f:
        data = line.split(',')
        frameNumber = data[0]
        data = data[1:]
        for i in range((len(data) - 1) // 7):
            T.append(float(data[7 * i]))
            W.append(np.array(
                    [
                        float(data[7 * i + 1]),
                        float(data[7 * i + 2]),
                        float(data[7 * i + 3])
                    ], dtype = np.float32))
            
            A.append(np.array(
                    [
                        float(data[7 * i + 4]),
                        float(data[7 * i + 5]),
                        float(data[7 * i + 6])
                    ], dtype = np.float32))

    OMEGA = np.stack(W, axis = 0)
    ACCEL = np.stack(A, axis = 0)

    fig, axes = plt.subplots(2, 3, figsize = (12, 8))
    fig.title("Time vs IMU Readings for Flight")
    axes[0][0].plot(T, OMEGA[0])
    axes[0][0].xlabel("Time (s)")
    axes[0][0].ylabel("w[0]")

    axes[0][1].plot(T, OMEGA[1])
    axes[0][1].xlabel("Time (s)")
    axes[0][1].ylabel("w[1]")

    axes[0][2].plot(T, OMEGA[2])
    axes[0][2].xlabel("Time (s)")
    axes[0][2].ylabel("w[2]")

    axes[1][0].plot(T, ACCEL[0])
    axes[1][0].xlabel("Time (s)")
    axes[1][0].ylabel("a[0]")

    axes[1][1].plot(T, ACCEL[1])
    axes[1][1].xlabel("Time (s)")
    axes[1][1].ylabel("w[1]")

    axes[1][2].plot(T, ACCEL[2])
    axes[1][2].xlabel("Time (s)")
    axes[1][2].ylabel("a[2]")

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", help = "Choice of the dataset to visualize")
    parser.add_argument("--path", help = "Path to the dataset to visualize")
    args = parser.parse_args()
    
    if args.data == "imu":
        visualizeIMU(args.path)
