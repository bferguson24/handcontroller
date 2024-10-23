import numpy as np




def deg2torque(theta1deg,theta2deg,theta3deg,W1,W2):
    #Parameters
    theta1 = np.deg2rad(theta1deg)
    theta2 = np.deg2rad(theta2deg)
    theta3 = np.deg2rad(theta3deg)
   
   
    # Dimensions in mm
    l0 = 110.39
    l1 = 252.64
    l2 = 300

    #Distance to Centroids
    dH_01 = (0,np.pi,l0,theta1)
    dH_12 = (0,-np.pi/2,0,theta2)
    dH_23 = (l1,0,0,theta3)
    dH_34 = (l2, 0, 0, 0)

    def transformationMatrix(ai,alphai,di,thetai):
        T_i = np.array([[np.cos(thetai),-np.sin(thetai),0,ai],
                        [np.sin(thetai)*np.cos(alphai),np.cos(thetai)*np.cos(alphai),-np.sin(alphai),-np.sin(alphai)*di],
                        [np.sin(thetai)*np.sin(alphai),np.cos(thetai)*np.sin(alphai),np.cos(alphai),np.cos(alphai)*di],
                        [0,0,0,1]])
        
        return T_i
    def zi(matrix):
        zi =np.array([matrix[0,2],matrix[1,2],matrix[2,2]])
        return zi
    
    T01 = transformationMatrix(*dH_01)
    T12 = transformationMatrix(*dH_12)
    T23 = transformationMatrix(*dH_23)
    T34 = transformationMatrix(*dH_34)
    
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    z1 = zi(T01)
    z2 = zi(T02)
    z3 = zi(T03)
    #Frame Origin Locations
    O = np.array([0,0,0])
    A = np.array([T02[0,3],T02[1,3],T02[2,3]])
    B = np.array([T03[0,3],T03[1,3],T03[2,3]])
    C = np.array([T04[0,3],T04[1,3],T04[2,3]])
  
    def jointTorque(Wi, zi, ri_start, ri_end):
        ri = ri_end - ri_start
        Fi = Wi * np.array([0, 0, -1])
        
        Ti = (Fi @ np.cross(zi,ri))
        return Ti


    #Distance of Centroid from axis origin
    #Length in m
    
    lc1 = 126.32
    lc2 = 150
    c1 = T02 @ np.array([lc1, 0,0,1])
    c1 = c1[:-1]
    c2 = T03 @ np.array([lc2,0,0,1])
    c2 = c2[:-1]
    T1 = 0
    T2 = jointTorque(W1,z2,A,c1) + jointTorque(W2,z2,A,c2)  #+ jointTorque(W3,z2,A,C))
    T3 = (jointTorque(W2,z3,B,c2)) #+ jointTorque(W3,z3,B,C))
    #print(T1,T2,T3)

    # Torque Scaling
    Tmax = l1*W1 + (l1+l2)*W2
    T1 = T1/Tmax
    T2 = -T2/Tmax
    T3 = -T3/Tmax

    # T1 = round(T1,2)
    # T2 = round(T2,2)
    # T3 = round(T3,2)
    #print(T1,T2,T3)

    #End Effector Position
    x,y,z = C[0],C[1],C[2]


    return T1, T2, T3,x,y,z


  # def plot_vectors_3d(vectors,titleName,colors=None):
    
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')
        
    #     for i, vector in enumerate(vectors):
    #         start, stop = vector
    #         x_start, y_start, z_start = start
    #         x_stop, y_stop, z_stop = stop
            
    #         color = colors[i] if colors and i < len(colors) else 'b'  # Default to blue if no color provided
    #         # Plot the vector
    #         ax.quiver(x_start, y_start, z_start, x_stop - x_start, y_stop - y_start, z_stop - z_start, 
    #                 color=color,arrow_length_ratio=0.05)
        
    #     # Set labels
    #     ax.set_xlabel('X')
    #     ax.set_ylabel('Y')
    #     ax.set_zlabel('Z')
        
    #     # Automatically scale axes to fit all vectors
    #     all_points = np.array([point for vector in vectors for point in vector])
    #     ax.set_xlim([all_points[:, 0].min() - 100, all_points[:, 0].max() + 100])
    #     ax.set_ylim([all_points[:, 1].min() - 100, all_points[:, 1].max() + 100])
    #     ax.set_zlim([all_points[:, 2].min() - 100, all_points[:, 2].max() + 100])
        
    #     ax.view_init(elev=45, azim=-45)
    #     ax.set_title(titleName)
    #     ax.grid(True)
    #     ax.view_init(elev=45, azim=-45)
    #     plt.show()
    # #Links, Joints
    # vectors = [(O,A),
    #         (A,B),
    #         (B,C),
    #         (A,(z1+A)),
    #         (A,(z2+A)),
    #         (B,(z3+B))]
    # colors = ['b','b','b','r','r','r']  # Colors for each vector

    # # print('Position of End Effector: ',(C))