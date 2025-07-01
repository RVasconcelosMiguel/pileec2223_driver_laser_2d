import rospy
from pathlib import Path
import rosbag
import matplotlib.pyplot as plt
import math
import numpy as np

bag_file = '/home/rodrigo/bagfiles2/rot2.bag'
topic1 = '/unnamed_robot/laser_scan_point_cloud'
topic2 = '/unnamed_robot/laser_scan_point_cloud2'
topic3 = '/unnamed_robot/cmd_vel'

bag = rosbag.Bag(bag_file)

x_coords1 = []
y_coords1 = []
x_coords2 = []
y_coords2 = []
errototal = []
velang = []
ids=1
erro=0
#(entre 1 e 381)  75, 120, 350 || comeca rotacao em 33 e termina em 354
idsout=0
ts1=0
aux=1

x = [1]
y = [1]

#plt.subplot(2,2,1)
plt.scatter(x, y, c='b', alpha=0.5)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Nuvem de pontos')

# plt.subplot(2,2,3)
# plt.scatter(x, y, c='r', alpha=0.5)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('NO CORRECTION POINT CLOUD')

# plt.subplot(2,2,2)
# plt.bar(0, errototal)
# plt.xticks(np.arange(len(errototal)))
# plt.xlabel('Index')
# plt.ylabel('Value')
# plt.title('ERRO')

# plt.subplot(2,2,4)
# plt.bar(0, velang)
# plt.xticks(np.arange(len(velang)))
# plt.xlabel('Index')
# plt.ylabel('Value')
# plt.title('ANGULAR VEL')


for j in range(300):
    idsout+=1
    if j==280:#280
        #CORRECT
        ids=1
        for topic, msg, t in bag.read_messages(topics=topic1):
            ids=ids+1
            if topic == topic1 and ids==idsout:
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

        for i in range(len(x_coords1)):
            erro = erro + ( ( x_coords1[ i ] - x_coords2[ i ] )**2 + ( y_coords1[ i ] - y_coords2[ i ] )**2 )**0.5

        if math.isnan(erro):
            erro=0

        errototal.append(erro) 
        erro=0
        
        aux=1
        for topic, msg, t in bag.read_messages(topics=topic3):
            if topic == topic3 and abs(t.to_sec()-ts1)<0.10 and aux==1:
                #print(t.to_sec())
                aux=0
                angular_velocity = msg.angular.z
                print(angular_velocity)
                velang.append(angular_velocity)
                break




    #CORRECTED POINT CLOUD
    #plt.subplot(2,2,1)
    
        plt.cla()
        plt.scatter(x_coords1, y_coords1, s=1, c='b', alpha=0.5, label='CORRECTED POINT CLOUD')
        plt.scatter(x_coords2, y_coords2, s=1, c='r', alpha=0.5, label='NO CORRECTION POINT CLOUD')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Nuvem de pontos')
        print(j)
        plt.xlim(-6, 6)
        plt.ylim(-6, 6)
        plt.pause(0.001)

    #NO CORRECTION POINT CLOUD
    # plt.subplot(2,2,3)
    # plt.cla()
    # plt.scatter(x_coords2, y_coords2, s=1, c='r', alpha=0.5, label='NO CORRECTION POINT CLOUD')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.title('NOCORRECTION')
    # plt.xlim(-6, 6)
    # plt.ylim(-6, 6)

    #ERRO
    # plt.subplot(2,2,2)
    # plt.cla()
    # plt.bar(np.arange(len(errototal)), errototal)
    # plt.xlabel('Index')
    # plt.ylabel('Value')
    # plt.title('ERROR')

    # #ANGULAR VELOCITY
    # plt.subplot(2,2,4)
    # plt.cla()
    # plt.bar(np.arange(len(velang)), velang)
    # plt.xlabel('Index')
    # plt.ylabel('Value')
    # plt.title('ANGULAR VELOCITY')

    x_coords1.clear()
    y_coords1.clear()
    x_coords2.clear()
    y_coords2.clear()
            
bag.close()

plt.show()



#/unnamed_robot/laser_scan_point_cloud
#/home/rodrigo/bagfiles/quarto.bag
