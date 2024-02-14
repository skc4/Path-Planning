# The Code Model for Path Planning
> Advanced path planning for autonomous vehicles using Frenet coordinates and spline interpolation.

This project demonstrates a sophisticated path planning system designed to navigate a vehicle around a track using sparse waypoints, sensor fusion data, and smooth trajectory generation. By employing Frenet coordinates and spline interpolation, the model addresses common issues like acceleration and jerk, ensuring smooth and efficient vehicle motion.

![Path Planning Visualization](https://github.com/kangcshin/Path-Planning/blob/master/Screenshot_from_2017-08-15_15-00-37.png)

### Track Complexity
The drive track is designed as a loop with varying densities of waypoints, posing unique challenges for maintaining consistent velocity and minimizing jerk due to the uneven distribution of track coordinates.

### Frenet Coordinates
Utilizing **Frenet coordinates** for navigation, the model distinguishes between longitudinal displacement (S) and lateral displacement (D) from the track's center, enabling more nuanced control over the vehicle's position and movements.

### Smoothing Movements
**Spline interpolation** is applied to connect waypoints smoothly, preventing abrupt changes in acceleration and direction that could lead to jerkiness, thus enhancing the comfort and safety of the vehicle's path.

### Generating Drive Path
The path generation process leverages **sensor fusion data**, incorporating information about the positions and velocities of surrounding vehicles. This data informs decisions on the vehicle's speed adjustments and lane changes to maintain safety and efficiency.

### Lane Analysis and Decision Making
The model intelligently divides the track into three lanes and makes dynamic lane-changing decisions based on the traffic conditions detected through sensor fusion. This strategic consideration ensures optimal lane selection for speed and safety.

### Implementation Highlights
- Starts in the middle lane and dynamically adjusts to traffic conditions, prioritizing safety and efficiency.
- Implements lane changing logic based on real-time traffic analysis, smoothly transitioning lanes when appropriate.
- Successfully navigates the track, demonstrating the capability to drive autonomously for over 4.32 miles without incidents.

### Watch the Drive
Experience the autonomous navigation in action: [Path Planning Video](https://youtu.be/9-lzesHC8Uk)

### Future Directions
- **Enhanced Traffic Prediction**: Integrating more advanced ML models to predict the behaviors of surrounding vehicles for improved decision-making.
- **Optimization Techniques**: Refining the spline interpolation and sensor fusion algorithms for even smoother path planning and better handling of complex traffic scenarios.
- **Cross-Functional Collaboration**: Exploring opportunities to apply NLP for interpreting and responding to dynamic road conditions and signals as part of a comprehensive autonomous driving system.
