# authors: David Afik, Daniel Penkov.

import numpy as np
import cv2 as cv
from Calibration import Calibration
from Position import Position
from KalmanFilter import KalmanFilter

class ArucoLocalizer:
    '''
    Aruco Localizer gives the position based on an Aruco in the space.
    '''
    def __init__(self, calib : Calibration, arucoID, arucoSize):
        '''
        Constractor, get calibration data, Aruco ID to lacalize by, and its dimention.
        '''

        # Tello camera points 12 degree downwards
        cameraFixedAngle = 12
        self.cameraFixedRad = np.deg2rad(cameraFixedAngle)

        # setup
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        # Set detection parameters
        detectorParams = cv.aruco.DetectorParameters()
        # Setup Aruco detector
        self.detector = cv.aruco.ArucoDetector(dictionary, detectorParams)

        self._arucoID = arucoID
        self._arucoSize = arucoSize
        self._calib = calib

            
        # Create Kalman filter instance------------------------------------------------------------------------------
        initial_theta = 0  # Initial estimate of theta
        initial_estimate_error_theta = 1  # Initial estimate error for theta
        process_variance_theta = 0.01  # Process noise variance for theta
        measurement_variance_theta = 0.1  # Measurement noise variance for theta
        self.kalman_theta = KalmanFilter(initial_theta, initial_estimate_error_theta, process_variance_theta, measurement_variance_theta)


    
    def getPosition(self, img : cv.Mat):
        '''
        Get the position of the camera in relation to the Aruco.
        '''

        # Detect all of the Arucos in the image.
        markerCorners, markerIds, _ = self.detector.detectMarkers(img)
        
        # If no Aruco detected.
        if markerIds is None:
            return None


        # Find the specific Aruco ID that we want.
        targetIndex = None
        for index, id in enumerate(markerIds):
            if id == self._arucoID:
                targetIndex = index
                break
        
        # If that ID was not found.
        if targetIndex == None:
            return None
        
        # Extract the position of the Aruco in relation to the camera.
        # rvecs is the rotation, and tvecs is the translasion.
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                                            markerCorners[targetIndex],
                                            self._arucoSize,
                                            self._calib.getIntrinsicMatrix(),
                                            self._calib.getExtrinsicMatrix())
        
        # If position extraction failed
        if rvecs is None:
            return None
        
        # Extract the rotation and translate vectors
        rvec = rvecs[0,0]
        tvec = tvecs[0].transpose()
        
        # Rotation matrix based on the aruco angle
        rmat, _ = cv.Rodrigues(rvec)
        rmat = np.matrix(rmat)

        # Calculate the aruco pose
        zvec = np.matrix([[0.], [0.], [-1.]])
        pvec = rmat * zvec

        tx = tvec[0,0]
        ty = tvec[1,0]
        tz = tvec[2,0]

        # Calculate the angle on XZ plan - out theta
        rad = -np.arctan2(pvec[0,0], pvec[2,0])
        theta = np.rad2deg(rad)

        # Update the Kalman filter with the new measurement-----------------------------------------------------------------
        #theta = self.kalman_theta.update(theta)
  
        # Calculate the position of the camera, in relation to the Aruco.
        x = -tx*np.cos(rad)-tz*np.sin(rad)
        y = ty*np.cos(self.cameraFixedRad) + tz*np.sin(self.cameraFixedRad)
        z = tx*np.sin(rad)-tz*np.cos(rad)
      
        #x = self.kalman_theta.update(x)

        pos = Position([round(x, 4), round(y, 4), round(z, 4)], round(theta, 4))
        # print("curr position: ", pos.toString())

        return pos
    

    def setAruco(self, arucoID : int, arucoSize : float):
        """
        set new Aruco details.

        Args:
            arucoID (int): code of Aruco
            arucoSize (float): size of the Aruco
        """
        self._arucoID = arucoID
        self._arucoSize = arucoSize



## test ##
if __name__ == "__main__":
    import VCS
    from djitellopy import Tello
    from DrawAruco import DrawAruco

    arucoId = 777
    arucoSize = 15.0

    # address = '10.42.0.127'
    # vport = 11111

    # tello = Tello(address)
    # tello.connect()
    # tello.set_network_ports(8890, vport)
    # tello.set_video_bitrate(Tello.BITRATE_1MBPS)
    # tello.streamon()

    #cam = VCS.VideoCapture("udp://" + address + ":" + str(vport))
    cam = VCS.VideoCapture(0)
    calib = Calibration("Camera Calibration/CalibDavid/Calibration.npy")
    AL = ArucoLocalizer(calib, arucoId, arucoSize)
    drawer = DrawAruco()

    while  cv.waitKey(50) != ord('q'):
        ret, img = cam.read()
        drawer.showImg(img)
        
        if not ret:
            exit(1)

        pos = AL.getPosition(img)
        if pos != None:
            print("curr position: ", pos.toString())
        else:
            print("None")

