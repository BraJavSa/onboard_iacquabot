\section{ROS\,2 Topic Listing}
\label{app:ros2_topics}
Table~\ref{tab:ros2_topics} lists all ROS\,2 topics published or subscribed by IAcquaBot, including message types, direction, nominal frequency, and a brief description.
\begin{table}[!t]
\caption{ROS\,2 Topics of the IAcquaBot System}
\label{tab:ros2_topics}
\centering
\tiny
\renewcommand{\arraystretch}{1.2}
\begin{tabular}{@{}llccp{2.5cm}@{}}
\toprule
\textbf{Topic} & \textbf{Message type} & \textbf{Dir.} & \textbf{Hz} & \textbf{Description} \\
\midrule
\multicolumn{5}{l}{\textit{PX4 Autopilot (MAVROS)}} \\
/mavros/local\_position/pose            & geometry\_msgs/PoseStamped   & Pub & 30 & Local position and orientation \\
/mavros/local\_position/velocity\_local & geometry\_msgs/TwistStamped  & Pub & 30 & Linear and angular velocities \\
/mavros/odometry/in                     & nav\_msgs/Odometry           & Pub & 30 & Full vehicle odometry \\
/mavros/imu/data                        & sensor\_msgs/Imu             & Pub & 50 & IMU acceleration and angular rate \\
/mavros/global\_position/global         & sensor\_msgs/NavSatFix       & Pub &  5 & GNSS position (lat, lon, alt) \\
/mavros/state                           & mavros\_msgs/State           & Pub &  1 & Autopilot state (mode, armed) \\
/mavros/manual\_control/control         & mavros\_msgs/ManualControl   & Pub & 20 & Radio manual commands \\
/mavros/setpoint\_velocity/cmd\_vel     & geometry\_msgs/TwistStamped  & Sub & 20 & Velocity setpoint \\
/mavros/setpoint\_position/local        & geometry\_msgs/PoseStamped   & Sub & 20 & Local position setpoint \\
/mavros/actuator\_control               & mavros\_msgs/ActuatorControl & Pub & 20 & Motor output commands \\
\midrule
\multicolumn{5}{l}{\textit{Microcontroller (micro-ROS)}} \\
/pwm\_outputs & std\_msgs/UInt16MultiArray & Sub & 20 & PWM signals $\in[1100,1900]$\,$\mu$s for 4 thrusters \\
\midrule
\multicolumn{5}{l}{\textit{Bathymetry Module}} \\
/lowrance/nmea\_raw          & std\_msgs/String             & Pub & -- & Raw NMEA sentences \\
/lowrance/gps/fix            & sensor\_msgs/NavSatFix       & Pub & -- & GPS fix \\
/lowrance/gps/vel            & geometry\_msgs/TwistStamped  & Pub & -- & Speed and heading \\
/lowrance/depth              & std\_msgs/Float32            & Pub & -- & Water depth (m) \\
/lowrance/water\_temp        & std\_msgs/Float32            & Pub & -- & Water temperature (°C) \\
/lowrance/heading/mag\_deg   & std\_msgs/Float64            & Pub & -- & Magnetic heading \\
/lowrance/heading/true\_deg  & std\_msgs/Float64            & Pub & -- & True heading \\
\midrule
\multicolumn{5}{l}{\textit{Surface Perception Module}} \\
/livox/lidar & sensor\_msgs/PointCloud2 & Pub & 10  & LiDAR point cloud \\
/livox/imu   & sensor\_msgs/Imu         & Pub & 200 & LiDAR integrated IMU \\
/iacquabot/sensors/camera/image\_raw & sensor\_msgs/Image & Pub & 15 & Raw RGB camera image \\
\bottomrule
\end{tabular}
\end{table}