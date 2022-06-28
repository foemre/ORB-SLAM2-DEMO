import numpy as np
import cv2
import glob
import yaml
import sys

class CameraCalibrator:
    def __init__(self, width=752, height=480, fps=30, num_images=100, pattern_size=(7,9), square_size=20, max_iter=30, eps=1e-5):
        self.width = width
        self.height = height
        self.fps = fps  
        self.num_images = num_images
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.max_iter = max_iter
        self.eps = eps
    def calibrate(self):
        __g_str = ("nvarguscamerasrc ! video/x-raw(memory:NVMM)," \
              "width=(int)1920, height=(int)1080, format=(string)NV12, " \
              "framerate=(fraction){}/1 ! nvvidconv ! video/x-raw, " \
              "width=(int){}, height=(int){}, format=(string)BGRx ! " \
              "videoconvert ! video/x-raw, format=(string)BGR !" \
              "appsink").format(self.fps, self.width, self.height)

        __cap = cv2.VideoCapture(__g_str, cv2.CAP_GSTREAMER)
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, self.max_iter, self.eps)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.pattern_size[0]*self.pattern_size[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:self.pattern_size[0],0:self.pattern_size[1]].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        found = 0
        for i in range(self.num_images):
            _, img  = __cap.read()
            #print(images[im_i])
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, self.pattern_size, corners2, ret)
                found += 1
             
                #cv2.waitKey(500)
                # if you want to save images with detected corners 
                # uncomment following 2 lines and lines 5, 18 and 19
                # image_name = path + '/calibresult' + str(found) + '.png'
                # cv2.imwrite(image_name, img)
            cv2.imshow('img', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        print("Number of images used for calibration: ", found)
        __cap.release()
        cv2.destroyAllWindows()
        print("Calibrating...")
        # calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        print("Calibration done")
        return ret, mtx, dist, rvecs, tvecs, objpoints, imgpoints

    def reprojError(self, ret, mtx, dist, rvecs, tvecs, objpoints, imgpoints):
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        total_error = mean_error/len(objpoints)
        return total_error
    def writeToFile(self, ret, mtx, dist, rvecs, tvecs, objpoints, imgpoints):
        # transform the matrix and distortion coefficients to writable lists
        data = {'camera_matrix': np.asarray(mtx).tolist(),
                'dist_coeff': np.asarray(dist).tolist()}
        print("Writing to calibration_matrix.yaml")
        # and save it to a file
        with open("calibration_matrix.yaml", "w") as f:
            yaml.dump(data, f)
