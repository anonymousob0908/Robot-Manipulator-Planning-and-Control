# Robot-Manipulator-Planning-and-Control
Trajectory optimization for robotic arms incorporating irregular-shaped obstacle avoidance using the GJK algorithm. Implemented joint-space tracking via PID, LQR, MPC, sliding mode, or high-gain high-frequency control.

##############################################################################################################################################################################################

Based on trajectory optimization for robotic arms, a supplement for obstacle avoidance of irregular objects has been implemented. Unlike spherical or cubical obstacles, avoiding irregularly shaped obstacles requires the use of the GJK algorithm. GJK algorithm MATLAB implementation reference: https://gitcode.com/open-source-toolkit/d45cf. After completing the trajectory optimization, simple joint-space control methods such as PID, LQR, MPC, sliding mode control, and high-gain high-frequency control are employed for trajectory tracking.
