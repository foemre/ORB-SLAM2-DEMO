import calibration

calibrator = calibration.CameraCalibrator(num_images = 30)
results = calibrator.calibrate()
print(calibrator.reprojError(*results))
calibrator.writeToFile(*results)