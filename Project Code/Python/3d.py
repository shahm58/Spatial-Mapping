# import open3d as o3d
# import numpy as np

# if __name__ == "__main__":

#     data = np.genfromtxt("tof_radar.xyz",delimiter=",")

#     # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(data)
#     o3d.visualization.draw_geometries([pcd])

#     alpha = 0.03
#     mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
#     mesh.compute_vertex_normals()
#     o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    


## Title O3D Create Test Data
#
#   Purpose: This example Python program simulates data received from a
#   sensor by writing the expected format of example data to a file. The
#   visualization will be demonstrated using a Python module called Open3D.
#
#   Special notes:
#       1. Open3D only works with Pythons 3.6-3.9.  It does not work with 3.10
#       2. For this eample you should run it in IDLE.  Anaconda/Conda/Jupyter
#       require different Open3D graphing methods (these methods are poorly documented)
#       3. Under Windows 10 you may need to install the MS Visual C++ Redistributable bundle
#           https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170
#       4. VirtualBox does not support OpenGL. If you're running Windows under Virtualbox or
#       your system doesn't support OpenGL (very rare), then you can install an OpenGL emulator dll
#           https://fdossena.com/?p=mesa/index.frag (unzip and copy opengl32.dll into Python dir)
#
#   T. Doyle
#   March 18, 2022 (Updated 2020 example)


import numpy as np
import open3d as o3d
if __name__ == "__main__":
    # #Remember the goals of modularization
    # #   -- smaller problems, reuse, validation, debugging
    # #To simulate the data from the sensor lets create a new file with test data 
    # f = open("demofile2dx.xyz", "w")    #create a new file for writing 
    
    # #Test data: Lets make a rectangular prism as a point cloud in XYZ format
    # #   A simple prism would only require 8 vertices, however we
    # #   will sample the prism along its x-axis a total of 10x
    # #   4 vertices repeated 10x = 40 vertices
    # #   This for-loop generates our test data in xyz format
    # for x in range(10):
    #     f.write('{0:d} 0 0\n'.format(x))    #write x,0,0 (xyz) to file as p1
    #     f.write('{0:d} 0 1\n'.format(x))    #write x,0,1 (xyz) to file as p2
    #     f.write('{0:d} 1 1\n'.format(x))    #write x,1,1 (xyz) to file as p3
    #     f.write('{0:d} 1 0\n'.format(x))    #write x,1,0 (xyz) to file as p4
    # f.close()   #there should now be a file containing 40 vertex coordinates                               
    
    # #Read the test data in from the file we created        
    # print("Read in the prism point cloud data (pcd)")
    # pcd = o3d.io.read_point_cloud("demofile2dx.xyz", format="xyz")
    
    data = np.genfromtxt("tof_radar.xyz",delimiter=",")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Lets see what our point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])

    #OK, good, but not great, lets add some lines to connect the vertices
    #   For creating a lineset we will need to tell the packahe which vertices need connected
    #   Remember each vertex actually contains one x,y,z coordinate

    measurements_per_spin = 30
    planes = 5

    #Give each vertex a unique number
    yz_slice_vertex = []
    for x in range(0,measurements_per_spin*planes):
        yz_slice_vertex.append([x])

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(0,measurements_per_spin*planes,measurements_per_spin):
        for i in range(0,measurements_per_spin-1):
            lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+1]])
        lines.append([yz_slice_vertex[x+measurements_per_spin-1], yz_slice_vertex[x]])

    #Define coordinates to connect lines between current and next yz slice        
    for x in range(0,measurements_per_spin*(planes-1),measurements_per_spin):
        for i in range(0,measurements_per_spin):
            lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+measurements_per_spin]])

    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])