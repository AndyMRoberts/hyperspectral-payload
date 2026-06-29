#Build:
$mkdir build
$cd build
$sudo cmake ..
$sudo make

#Run:
$./stereo_calibration -w=9 -h=6 -s=35 ../images/stereo_calib.xml

#RES
see the in/extrinsics results in "cal_results" folder, copyt them into your own project folder


Results: 1280x720
31 pairs have been successfully detected.
Running stereo calibration ...
done with RMS error=4.92013
average epipolar err = 6.49025



