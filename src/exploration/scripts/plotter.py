#!/usr/bin/env python
import rospy
import numpy as num
from matplotlib import pyplot as plt
from exploration.srv import jednad, dvad


def plot_it(req):

    plt.plot(req)
    plt.show()


def plot_server():
    rospy.init_node('plot_server')
    rospy.Service('plotXY', dvad, plot_it)
    rospy.spin()

if __name__ == "__main__":
    plot_server()