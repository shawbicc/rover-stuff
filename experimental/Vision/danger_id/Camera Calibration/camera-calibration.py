#!/usr/bin/env python3
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt 

def calibrate(showPics=True):
    imgPathList = []
    for i in range(16):
        imgPathList.append(f"..//my camera calibration//{i + 1}.jpg")

    # Initialize  
    nRows = 9 
    nCols = 6 
    termCriteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,30,0.001)
    worldPtsCur = np.zeros((nRows*nCols,3), np.float32)
    worldPtsCur[:,:2] = np.mgrid[0:nRows,0:nCols].T.reshape(-1,2)
    worldPtsList = []
    imgPtsList = [] 

    # Find Corners 
    for curImgPath in imgPathList:
        imgBGR = cv.imread(curImgPath)
        # imgBGR = cv.flip(imgBGR, 1)
        imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)
        cornersFound, cornersOrg = cv.findChessboardCorners(imgGray,(nRows,nCols), None)

        if cornersFound == True:
            worldPtsList.append(worldPtsCur)
            cornersRefined = cv.cornerSubPix(imgGray,cornersOrg,(11,11),(-1,-1),termCriteria)
            imgPtsList.append(cornersRefined)

            if showPics: 
                cv.drawChessboardCorners(imgBGR,(nRows,nCols),cornersRefined,cornersFound)
                cv.imshow('Chessboard', imgBGR)
                cv.waitKey(500)
    cv.destroyAllWindows()

    # Calibrate 
    repError,camMatrix,distCoeff,rvecs,tvecs = cv.calibrateCamera(worldPtsList, imgPtsList, imgGray.shape[::-1],None,None)
    print('Camera Matrix:\n',camMatrix)
    print("Reproj Error (pixels): {:.4f}".format(repError))
    
    # Save Calibration Parameters (later video)
    # curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = '..//Camera Calibration//calibration.npz'
    np.savez(paramPath, 
        repError=repError, 
        camMatrix=camMatrix, 
        distCoeff=distCoeff, 
        rvecs=rvecs, 
        tvecs=tvecs)
    
    return camMatrix,distCoeff



def runCalibration(): 
    calibrate(showPics=True) 

if __name__ == '__main__': 
    runCalibration() 