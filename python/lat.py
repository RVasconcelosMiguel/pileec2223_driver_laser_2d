import rospy
from pathlib import Path
import rosbag
import matplotlib.pyplot as plt
import math
import numpy as np

bag_file = '/home/rodrigo/bagfiles/finais/lateral.bag'
topic1 = '/unnamed_robot/laser_scan_point_cloud'
topic2 = '/unnamed_robot/laser_scan_point_cloud2'
topic3 = '/unnamed_robot/cmd_vel'
numids = 170

bag = rosbag.Bag(bag_file)

x_coords1 = []
y_coords1 = []
x_coords2 = []
y_coords2 = []
errototal = np.zeros(numids)
velang = np.zeros(numids)
vellinx = np.zeros(numids)
velliny = np.zeros(numids)
ids=1
erro=0
idsout=0
ts1=0
aux=1

x = [1]
y = [1]

for j in range(numids):
    idsout+=1
    #CORRECT
    ids=1
    for topic, msg, t in bag.read_messages(topics=topic1):
        ids=ids+1
        if topic == topic1 and ids==idsout:
            ts1=t.to_sec()
            for point in msg.points:
                x_coords1.append(point.x)
                y_coords1.append(point.y)
                
    ids=1
    #NO CORRECTION
    for topic, msg, t in bag.read_messages(topics=topic2):
        ids=ids+1
        if topic == topic2 and ids==idsout:
            for point in msg.points:
                x_coords2.append(point.x)
                y_coords2.append(point.y)

    #EUCLIDEAN ERROR
    for i in range(len(x_coords1)):
        erro = erro + ( ( x_coords1[ i ] - x_coords2[ i ] )**2 + ( y_coords1[ i ] - y_coords2[ i ] )**2 )**0.5

    if math.isnan(erro):
        erro=0

    errototal[j]=erro
    erro=0
    
    #VELOCITY
    aux=1
    for topic, msg, t in bag.read_messages(topics=topic3):
        if topic == topic3 and abs(t.to_sec()-ts1)<0.10 and aux==1:
            #print(t.to_sec())
            aux=0          
            angular_velocity = msg.angular.z
            linear_velx = msg.linear.x
            linear_vely = msg.linear.y
            print(angular_velocity)
            if angular_velocity > 3:
                angular_velocity = 3
            if angular_velocity < -3:
                angular_velocity = -3
            velang[j]=angular_velocity
            vellinx[j] = linear_velx
            velliny[j] = linear_vely
            break

    #CORRECTED POINT CLOUD
    if j == 70:
        plt.subplot(2,3,1)
        plt.cla()
        plt.scatter(x_coords1, y_coords1, s=1, c='b', alpha=0.5, label='CORRECTED POINT CLOUD')
        plt.scatter(x_coords2, y_coords2, s=1, c='r', alpha=0.5, label='NO CORRECTION POINT CLOUD')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('IDS NUMBER 70 POINT CLOUD')
        plt.xlim(-6, 6)
        plt.ylim(-6, 6)
    plt.pause(0.001)

    #ERROR
    plt.subplot(2,3,4)
    plt.cla()
    plt.bar(np.arange(len(errototal)), errototal)
    plt.xlabel('IDS')
    plt.ylabel('Value')
    plt.title('EUCLIDEAN ERROR')

    #ANGULAR VELOCITY
    plt.subplot(2,3,3)
    plt.cla()
    plt.bar(np.arange(len(velang)), velang)
    plt.xlabel('IDS')
    plt.ylabel('rad/s')
    plt.title('ANGULAR VELOCITY')
    plt.ylim(-4, 4)

    #LINEAR X VELOCITY
    plt.subplot(2,3,2)
    plt.cla()
    plt.bar(np.arange(len(vellinx)), vellinx)
    plt.xlabel('IDS')
    plt.ylabel('m/s')
    plt.title('LINEAR X VELOCITY')
    plt.ylim(-1, 1)

    #LINEAR Y VELOCITY
    plt.subplot(2,3,5)
    plt.cla()
    plt.bar(np.arange(len(velliny)), velliny)
    plt.xlabel('IDS')
    plt.ylabel('m/s')
    plt.title('LINEAR Y VELOCITY')
    plt.ylim(-1, 1)

    x_coords1.clear()
    y_coords1.clear()
    x_coords2.clear()
    y_coords2.clear()
            
bag.close()

plt.show()



#/unnamed_robot/laser_scan_point_cloud
#/home/rodrigo/bagfiles/quarto.bag
