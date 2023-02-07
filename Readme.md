# Camera Calibration

## Monocular Calibration
```camera_calibration_mono.py```
Calculates the intrinsic camera matrix of all the defined image topics in a sequential manner and saves them in json file ```camera_intrinsics.json```.

## Stereo Calibration
```camera_stereo_calibration.py``` Performs the stereo calibration on the given image topic pairs with the pattern board placed static. It requires intrinsic calibration file generated previously and saves extrinsic properties in ```camera_extrinsic.json``` 

## Calibration with centre of car
```calibration_to_coc.py``` Performs the ```cv2.solvePnPRansac``` on the given image topics to find the relative pose wrt to coc. The position of calibration board wrt to coc (``` T_board and  R_board``` )should be defined. Calibration is performed with fixed board pose.
