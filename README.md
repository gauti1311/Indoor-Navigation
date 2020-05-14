# Indoor-AR-Navigation

The project is to use simple datamatrix codes placed in the environment to estimate the robot position in the given map by means of triangulation and some filters to improve the estimations. 
The approach consists of the following main steps:
1. Prepare data matrix (markers) to pace in the environment used as landmarks.
2. Develop algorithms to perform robot localization based on the visual information extracted from cameras and estimation techniques for enhanced robustness (such as Kalman Filters). 
3. Define dedicate source and target points on the environment and implement some appropriate path planning techniques.
4. Perform robot motion control along the planned path by using the localization information extracted from the passive markers.
5. Implement some solution to prevent collisions, for example laser scaner can be used as a solution to try obstacle detection.

# python modules 
1. numpy
2. PIL (as an alternative of OpenCV)
# Dependancy other than python modules
1. Python wrapper of Libdmtx from ----------
2. Replace original pylibdmtx.py with given in this repo from root folder ----- to add all 4 corners.
3. CV_bridge to convert raw images in OpenCV image.
