from pathlib import Path
import rosbag
import matplotlib.pyplot as plt
import rospy
import math
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from pathlib import Path
from rosbags.bag import Bag

bag_file = [Path('/home/rodrigo/bagfiles/quarto.bag')]
topic1 = '/unnamed_robot/laser_scan_point_cloud'
topic2 = '/unnamed_robot/laser_scan_point_cloud2'
topic3 = '/unnamed_robot/cmd_vel'

#bag = rosbag.Bag(bag_file)

x_coords1 = []
y_coords1 = []
x_coords2 = []
y_coords2 = []
errototal = []
ids=1
erro=0
#(entre 1 e 381)  75, 120, 350 || comeca rotacao em 33 e termina em 354
idsout=0

x = np.random.rand(100)
y = np.random.rand(100)

plt.subplot(2,2,1)
plt.scatter(x, y, c='b', alpha=0.5)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('CORRECTED POINT CLOUD')

plt.subplot(2,2,3)
plt.scatter(x, y, c='r', alpha=0.5)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('NO CORRECTION POINT CLOUD')

plt.subplot(2,2,2)
plt.bar(0, errototal)
plt.xticks(np.arange(len(errototal)))
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('ERRO')


for j in range(380):
    idsout+=1
    #CORRECT
    ids=1
    with AnyReader(bag_file) as reader:
        ids = 1
        for topic, msg, t in reader.read_messages(topics=topic1):
            ids += 1
            if topic == topic1 and ids == idsout:
                timestamp = msg.header.stamp.to_sec()
                print("Timestamp:", timestamp)
                for point in msg.points:
                    x_coords1.append(point.x)
                    y_coords1.append(point.y)

                
    ids=1
    #NO CORRECTION
    with AnyReader(bag_file) as reader:
        ids = 1
        for topic, msg, t in reader.read_messages(topics=topic2):
            ids += 1
            if topic == topic1 and ids == idsout:
                timestamp = msg.header.stamp.to_sec()
                print("Timestamp:", timestamp)
                for point in msg.points:
                    x_coords1.append(point.x)
                    y_coords1.append(point.y)

    for i in range(len(x_coords1)):
        erro=erro+((x_coords1[i]-x_coords2[i])**2+(y_coords1[i]-y_coords2[i])**2)**0.5
    if math.isnan(erro):
        erro=0
    errototal.append(erro) 
    erro=0

    #CORRECTED POINT CLOUD
    plt.subplot(2,2,1)
    plt.cla()
    plt.scatter(x_coords1, y_coords1, s=1, c='b', alpha=0.5, label='CORRECTED POINT CLOUD')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('CORRECT')
    plt.xlim(-6, 6)
    plt.ylim(-6, 6)
    plt.pause(0.001)

    #NO CORRECTION POINT CLOUD
    plt.subplot(2,2,3)
    plt.cla()
    plt.scatter(x_coords2, y_coords2, s=1, c='r', alpha=0.5, label='NO CORRECTION POINT CLOUD')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('NOCORRECTION')
    plt.xlim(-6, 6)
    plt.ylim(-6, 6)

    #ERRO
    plt.subplot(2,2,2)
    plt.cla()
    plt.bar(np.arange(len(errototal)), errototal)
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('ERRO')

    x_coords1.clear()
    y_coords1.clear()
    x_coords2.clear()
    y_coords2.clear()
            
bag.close()

plt.show()



#/unnamed_robot/laser_scan_point_cloud
#/home/rodrigo/bagfiles/quarto.bag
