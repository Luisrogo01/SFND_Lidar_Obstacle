# Lidar Obstacle Detection

## Details
In this project given a set of Point Cloud Data from a Lidar Sensor I implemeted Segmentation and Clustering to detect objects around the vehicle.
The original point cloud given without pre-processing methods is:

![Image](https://media.giphy.com/media/LPfQ8v1TgXdLw07C1H/giphy.gif)

### Segmentation using RANSAC

RANSAC stands for Random Sample Consensus, and is a method for detecting outliers in data. RANSAC runs for a max number of iterations, and returns the model with the best fit. Each iteration randomly picks a subsample of the data and fits a model through it, such as a line or a plane. Then the iteration with the highest number of inliers or the lowest noise is used as the best model.
The result is as shown:

![Image](https://media.giphy.com/media/fZ8rZRkoaDuPyiHtUM/giphy.gif)

### Euclidean Clustering using KD-Tree

A KD-Tree is a binary tree that splits points between alternating axes. By separating space by splitting regions, nearest neighbor search can be made much faster when using an algorithm like euclidean clustering.
The final result is:

![Image](https://media.giphy.com/media/KFV4HgS7VEmrTbcwjJ/giphy.gif)

## Instalation
### Dependencies needed

* cmake >= 3.14
* gcc/g++ >= 8.0
* PCL >= 1.2

### Running
#### Ubuntu

```
$> cd ~
$> git clone https://github.com/Luisrogo01/SFND_Lidar_Obstacle.git
$> cd SFND_Lidar_Obstacle
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```
