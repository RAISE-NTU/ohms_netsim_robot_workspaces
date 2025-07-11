# **OHMS Robot ROS2 Workspace**

This repository contains the ROS2 workspace for the Outdoor Heterogeneous Multi-robot System (OHMS), designed for ROS2 Humble. It includes packages for managing robot communications, converting odometry data to TF transforms, and generating OctoMaps for navigation.

## **Packages**

This workspace contains the following packages:

* **ohms\_robot\_comms\_manager**: Manages communication between multiple robots in the OHMS system.  
* **odom\_to\_tf\_ros2**: Converts odometry data to TF transforms for use with other ROS2 packages.  
* **octomap\_server2**: A ROS2 wrapper for the OctoMap library, which generates 3D occupancy grids from point cloud data.  
* **perception\_pcl**: A ROS2 wrapper for the Point Cloud Library (PCL), providing tools for point cloud processing.

## **Prerequisites**

Before building the workspace, ensure you have the following installed:

* ROS2 Humble Hawksbill  
* Eigen3  
* Point Cloud Library (PCL)  
* OctoMap library

## **Building the Workspace**

To build the workspace, navigate to the root directory and run the following command:

```
colcon build --symlink-install
```

## **Running the System**

To run the system, you will need to launch the appropriate launch files for your specific robot. For example, to launch the communications subscriber for the atlas robot, you would use the following command:

```
ros2 launch ohms_robot_comms_manager atlas_ohms_robot_comms_subscriber.launch.py
```

This will start the ohms\_robot\_comms\_subscriber node, which handles communication with other robots in the network. You can then launch other necessary nodes for your robot, such as the odom\_to\_tf node and the octomap\_server node.

## **Package: odom\_to\_tf\_ros2**

The odom\_to\_tf\_ros2 package provides a node that converts nav\_msgs/msg/Odometry messages into TF transforms. This is essential for maintaining a consistent coordinate frame across all robots in the system.

### **Parameters**

The odom\_to\_tf node accepts the following parameters:

* **frame\_id**: The parent frame ID for the TF transform (e.g., map or odom).  
* **child\_frame\_id**: The child frame ID for the TF transform (e.g., base\_link).  
* **odom\_topic**: The name of the odometry topic to subscribe to.

### **Example Launch**

Here is an example of how to launch the odom\_to\_tf node for the atlas robot:

```
ros2 launch odom_to_tf_ros2 atlas_odom_to_tf.launch.py
```

This will launch the node, subscribing to the /atlas/odom\_ground\_truth topic. It will then publish TF transforms with the parent frame atlas/map and the child frame atlas/base\_link.

## **Package: ohms\_robot\_comms\_manager**

The ohms\_robot\_comms\_manager package provides nodes that manage communication between multiple robots in the OHMS system. It includes a subscriber to receive data from other robots and a publisher to broadcast its own link status.

### **Subscriber Node (ohms\_robot\_comms\_subscriber)**

The subscriber node listens for topics from other robots and republishes them locally with appropriate Quality of Service (QoS) settings to handle variable network conditions.

#### **Parameters**

* **robot\_name**: The name of the robot on which the node is running.  
* **other\_robots**: A list of other robots in the system to establish communication with.  
* **link\_quality\_threshold**: The minimum link quality required to subscribe to high-bandwidth topics like the full OctoMap.  
* **window\_size**: The size of the sliding window used to calculate the link quality.

#### **Example Launch**

The following command launches the ohms\_robot\_comms\_subscriber node for the atlas robot:

```
ros2 launch ohms_robot_comms_manager atlas_ohms_robot_comms_subscriber.launch.py
```

This will start the node, which will then subscribe to topics from other robots (as configured in the launch file) and republish them locally.

### **Publisher Node (ohms\_robot\_comms\_publisher)**

The publisher node is responsible for publishing internal topics with QoS updates.

#### **Parameters**

* **robot\_name**: The name of the robot on which the node is running.  

#### **Example Launch**

The following command launches the ohms\_robot\_comms\_publisher node for the atlas robot:

```
ros2 launch ohms_robot_comms_manager atlas_ohms_robot_comms_publisher.launch.py
```

This will start the node.

### **Octomap**

```
ros2 launch octomap_server2 atlas_octomap_server_launch.py  
```