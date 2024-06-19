#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from generate_tof_map.srv import GenerateHeatmap, GenerateHeatmapResponse


def handle_generate_heatmap(req):
     # Convert the flat data into a 2D numpy array
    data = np.array(req.data).reshape(req.rows, req.cols)
    
    fig, ax = plt.subplots()
    cax = ax.imshow(data, cmap='coolwarm', interpolation='nearest')
    
    # Add color bar
    cbar = fig.colorbar(cax)
    cbar.set_label('Distance Error (m)')

    # Annotate the heatmap
    for i in range(req.rows):
        for j in range(req.cols):
            ax.text(j, i, f"{data[i, j]:.3f}", ha='center', va='center', color='black')

    ax.set_title(req.name_graph)
    ax.set_xlabel("Pixel Column")
    ax.set_ylabel("Pixel Row")

    plt.show()

    response = GenerateHeatmapResponse()
    response.success = True
    response.message = "Heatmap displayed successfully."
    return response
    
def generate_heatmap_server():
    rospy.init_node('generate_heatmap_server')
    s = rospy.Service('generate_heatmap', GenerateHeatmap, handle_generate_heatmap)
    print("Ready to generate heatmaps.")
    rospy.spin()

if __name__ == "__main__":
    generate_heatmap_server()
