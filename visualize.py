#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

def visualization():
	# load csv file and plot trajectory
	_, ax = plt.subplots(1)
	ax.set_aspect('equal')
	ax.grid(True)

	trajectory = np.loadtxt("trajectory.csv", delimiter=',')
	plt.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2)

	plt.xlim(0, 5)
	plt.ylim(-2, 3)
	plt.xlabel('Global Space X (m)')
	plt.ylabel('Global Space Y (m)')
	plt.title('Turtlebot Motion Path - TC 1')
	plt.show()

if __name__ == '__main__':
	visualization()