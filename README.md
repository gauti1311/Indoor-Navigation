# Indoor-AR-Navigation

The project is to use simple datamatrix codes placed in the environment to estimate the robot position in the given map by means of triangulation and some filters to improve the estimations. 
The approach consists of the following main steps:
1. Prepare data matris (markers) to pace in the environment 
2. Develop algorithms to perform robot localization based on the visual information extracted from cameras and estimation techniques for enhanced robustness (such as Kalman Filters). 
3. Define dedicate source and target points on the environment and implement some appropriate path planning techniques.
4. Perform robot motion control along the planned path by using the localization information extracted from the passive markers.
5. Implement some solution to prevent collisions, for example laser scaner can be used as a olution to try obstacle detection.
