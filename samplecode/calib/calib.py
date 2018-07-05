# stereo calibration example script

import sys, os, cv2
import numpy as np
np.set_printoptions(precision=25)

# please options below
# ** BEGIN OPTIONS **
# size of chessboard
chessboard_size = (7, 6)
chessboard_cell_meters = 0.05

# size of images (each image should be horizontally concatenated left/right camera images, total size 2*IMG_W, IMG_H)
IMG_H, IMG_W = 400, 640

# directory containing calibration images
root_dir = "calib"

# where to save results to (YAML format)
output_path = "calib.yaml"
# ** END OPTIONS **

CAT_W = 2 * IMG_W

files = os.listdir(root_dir)
ims = [cv2.imread(os.path.join(root_dir, f)) for f in files]

objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2)
objp *= chessboard_cell_meters
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def reproj_err(objpoints, imgpoints, mtx, dist, rvecs, tvecs):
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    return mean_error/len(objpoints)

def drawlines(img1,img2,lines,pts1,pts2):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r,c = img1.shape
    img1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
    img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv2.line(img1, (x0,y0), (x1,y1), color,1)
        img1 = cv2.circle(img1,tuple(pt1),5,color,-1)
        img2 = cv2.circle(img2,tuple(pt2),5,color,-1)
    return img1,img2

def flat_print(A):
    print("", np.array2string(A.flatten(), precision=20, separator=',')[1:-1])

print("Detecting chessboard corners...")
objpoints = [] # 3d point in real world space
imgpointsl = [] # 2d points in image plane.
imgpointsr = [] # 2d points in image plane.

for im in ims:
    left = im[:, :IMG_W, :]
    leftg = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    retl, cornersl = cv2.findChessboardCorners(leftg, (7,6), None)

    right = im[:, IMG_W:, :]
    rightg = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    retr, cornersr = cv2.findChessboardCorners(rightg, (7,6), None)

    if cornersl is not None and cornersr is not None:
        cornersl = cv2.cornerSubPix(leftg, cornersl, (11,11), (-1, -1), criteria)
        cornersr = cv2.cornerSubPix(rightg, cornersr, (11,11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpointsl.append(cornersl)
        imgpointsr.append(cornersr)

print("\nCalibrating cameras individually...")
criteria_ca = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 350, 1e-6)
print(" Calibrating left")
flag = cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
retl, mtxl, distl, rvecsl, tvecsl = cv2.calibrateCamera(objpoints, imgpointsl, (IMG_W, IMG_H), None, None, flags=flag, criteria = criteria_ca)
print(" Calibrating right")
retr, mtxr, distr, rvecsr, tvecsr = cv2.calibrateCamera(objpoints, imgpointsr, (IMG_W, IMG_H), None, None, flags=flag, criteria = criteria_ca)

print("> cam_matrix_left")
flat_print(mtxl)
print("> cam_matrix_right")
flat_print(mtxr)
print("> distortion_coeffs_left")
flat_print(distl)
print("> distortion_coeffs_right")
flat_print(distr)

print("\nPerforming stereo calibration...")
criteria_sc = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 350, 1e-6)
flags = cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6

(ret, _, _, _, _, rotationMatrix, translationVector, E, F) = cv2.stereoCalibrate(
        objpoints, imgpointsl, imgpointsr,
        mtxl, distl, mtxr, distr,
        (IMG_W, IMG_H), None, None, None, None,
        flags, criteria_sc)

print("Stereo calibration pixel error", ret)
print("Reprojection error left", reproj_err(objpoints, imgpointsl, mtxl, distl, rvecsl, tvecsl))
print("Reprojection error right", reproj_err(objpoints, imgpointsr, mtxr, distr, rvecsr, tvecsr))

print("\nPerforming stereo rectification...")
OPTIMIZE_ALPHA = -1
(R1, R2, P1, P2, Q, leftROI, rightROI) = cv2.stereoRectify(
                mtxl, distl,
                mtxr, distr,
                (IMG_W, IMG_H), rotationMatrix, translationVector,
                E, F, None, None, None
                , cv2.CALIB_ZERO_DISPARITY, -1, (IMG_W, IMG_H))

print("> R1")
flat_print(R1)
print("> R2")
flat_print(R2)
print("> P1")
flat_print(P1)
print("> P2")
flat_print(P2)
print("> Q")
flat_print(Q)

print("\n(not required for OpenARK)")
print("> rotationMatrix")
flat_print(rotationMatrix)
print("> translationvector")
flat_print(translationVector)

print("\nTesting, writing output...")
leftMapX, leftMapY = cv2.initUndistortRectifyMap(mtxl, distl, R1, P1, (IMG_W, IMG_H), cv2.CV_32FC1)
rightMapX, rightMapY = cv2.initUndistortRectifyMap(mtxr, distr, R2, P2, (IMG_W, IMG_H), cv2.CV_32FC1)

    
# debug output
idx = 0
iml = ims[idx][:, :IMG_W, :]
imr = ims[idx][:, IMG_W:, :]
dstl = cv2.undistort(iml, mtxl, distl, None, None)
dstr = cv2.undistort(imr, mtxr, distr, None, None)

leftFrame = ims[idx][:, :IMG_W, :]
fixedLeft = cv2.remap(leftFrame, leftMapX, leftMapY, cv2.INTER_LINEAR)

rightFrame = ims[idx][:, IMG_W:, :]
fixedRight = cv2.remap(rightFrame, rightMapX, rightMapY, cv2.INTER_LINEAR)
fixed = np.hstack([fixedLeft, fixedRight])

output = {}
output["imageWidth"] = fixedLeft.shape[1]
output["imageHeight"] = fixedLeft.shape[0]
output["undistortedWidth"] = IMG_W
output["undistortedHeight"] = IMG_H
output["cameraMatrix1"] = mtxl
output["cameraMatrix2"] = mtxr
output["distCoeffs1"] = distl
output["distCoeffs2"] = distr
output["R1"] = R1
output["R2"] = R2
output["P1"] = P1
output["P2"] = P2
output["Q"] = Q

fs_write = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
for k in output:
    fs_write.write(k, output[k])
fs_write.release()
print("calibration written to file:", output_path)

for i in range(0, IMG_H, 20):
    cv2.line(fixed, (0,i), (CAT_W, i), (255, 0 ,0))

cv2.imshow("undistorted", np.hstack([dstl, dstr]))
cv2.imshow("original", ims[idx])
cv2.imshow("fix", fixed)
cv2.waitKey(0)
