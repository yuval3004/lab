from TelloControlAruco import TelloControlAruco as TelloControl
from SwarmControlAruco import SwarmControlAruco
from ArucoLocalizer import ArucoLocalizer
from Position import Position
from DrawAruco import DrawAruco
from TelloDetails import getTelloDetailsList
from time import sleep
import numpy as np
import cv2 as cv
import signal
import matplotlib.pyplot as plt
import threading

## test ##
if __name__ == "__main__":
    swarm = None

    def EMERGENCY_STOP_HANDLER(sig_num, frame):
        '''
        ### this method is to instantly land all of the drones in an emergency.
        #### to activate, press ctrl+c in the terminal.
        '''
        print()
        print(" ******************************************************************** ")
        print(" ************************** EMERGENCY STOP ************************** ")
        print(" ******************************************************************** ")
        swarm.kill()
        exit(0)



    def showPlt(tello : TelloControl, points):
        plt.ion()
        x, z, tmpx, tmpz = 0, 0, 0, 0
        fig, ax = plt.subplots()
        ax.set_xlim(-300, 300)
        ax.set_ylim(-400, -100)
        plt.xlabel('x - axis')
        plt.ylabel('z - axis')

        while swarm != None:
            pos = tello._currentPosition
 
            if pos is None:
                sleep(waitKey // 1000)
                continue
            
            tmpx = x
            tmpz = z
            ax.scatter([tmpx], [tmpz], alpha = 0.3, color= "blue", s=30) 

            x = pos.getX() # get the X from tello  
            z = pos.getZ() # get the Z from tello 
            y = pos.getY() # get the Y from tello 
            t = pos.getT() # get the Y from tello 
            points.append((x, y ,z, t))
            
        
            ax.scatter([x], [z], alpha = 0.7, color= "blue", s=30) 
    
            plt.pause(0.1)
            plt.cla()
            ax.set_xlim(-100, 100)
            ax.set_ylim(-400, -100)
            plt.xlabel('x - axis')
            plt.ylabel('z - axis')
       
            fig.canvas.draw()

    points = []

    # link the ctrl+c signal to our custom signal handler.
    signal.signal(signal.SIGINT, EMERGENCY_STOP_HANDLER)

    # time between the commands sent to the swarm, in ms.
    # i.e. if waitKey is 100ms, then the swarm will get advance orders 10 times in a second.
    waitKey = 100

    # maximum error of the swarm that we allow.
    okError = -15

    arucoId = 605
    arucoSize = 17.5

    drawer = DrawAruco()

    # list of the tellos that are participant in the swarm
    participants = getTelloDetailsList(5)

    # list of all the tellos in swarm
    tellos = [
        TelloControl(
            tello.address, 
            tello.vport, 
            ArucoLocalizer(tello.calib, 100, 15.0)
        ) for tello in participants]

    # the path relative to the Aruco.
    targets = [
        [[   0,  20,-300],   0],
        #[[   0,  20,-200],   0],
        #[[-70,  20,-170],  0],
        #[[70,  20,-170],  0],
        #[[50,  20, -170], 0],
        #-----
        #[[0,  20,-170], 0],
        # [[   0,  20,-170],  45],
        # [[   0,  20,-170],  90],
        # [[-200,  20,-240], 180],
        # [[-200,  20,-240], 225],
        # [[-200,  20,-240], 270],
        # [[-100,  20,-240], 270],
        # [[   0,  20,-240], 270],
        # [[   0,  20,-240], 315],
        # [[   0,  20,-240],   0],
        # [[   0,  20,-170],   0],
    ]

    # initialize SwarmControl.
    swarm = SwarmControlAruco(tellos, scale = 45)
    swarm.startCam()

    # 3 seconds count down before takeoff.
    print("taking off!!! in:")
    for i in range(3, 0, -1):
        print(i)
        sleep(1)

    swarm.takeOff()
    swarm.setTelloHeight(120)

    thread_plt = threading.Thread(target=showPlt, args=(tellos[0],points,))
    thread_plt.start()

    # set next Aruco as reference point.
    swarm.setAruco(arucoId, arucoSize)
    
    # rotate all of the drones until they find their Aruco.
    # swarm.findAruco()

    # move the drones along a path.
    for target in targets:
        # set the next point in the path as the new target.
        swarm.setTarget(Position(np.array(target[0]), target[1]))

        # keep advancing toward the target until you get small enough error.
        
        while swarm.getError() > okError and cv.waitKey(waitKey) is not ord('q') and len(points)<100 :
            print("error: ", swarm.getError())
            swarm.advance()
            drawer.showImg(tellos[0].getLastImage())
    
    filename = "fixedX_3m_data.txt"
    with open(filename, "w") as file:
    # Write each point to the file
        for point in points:
            x, y, z, t  = point
            file.write(f"{x}\t{y}\t{z}\t{t}\n")
    
    # end flight
    cv.destroyAllWindows()
    swarm.kill()

