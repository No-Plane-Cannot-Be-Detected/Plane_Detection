### Technical Background

Our team plans to implement the algorithm of **plane detection from 3D point cloud**, and deal with certain noise and isolated points.

Point cloud data refers to a set of vectors in a three-dimensional coordinate system, recorded in the form of points. The information of each point includes three-dimensional coordinates, and some may contain color information or reflection intensity information. For the algorithm of recognizing multiple planes in point cloud space, we only need to use the three-dimensional coordinate information of point cloud. We believe that a matrix [**cv::Mat**](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html)  with N rows (total number of data points) and 3 columns (three-dimensional coordinates) is used in OpenCV. As a data structure for storing point cloud, it has great speed advantage in matrix operation, and point cloud data is continuously stored in memory. It is convenient to operate the pointer.

After many considerations, our team decided to mainly use RANSAC algorithm to realize multi plane detection in point cloud space. In order to solve the problem of high computation of RANSAC algorithm in a large number of point cloud data, we will sample the original data of point cloud, reduce the amount of data of points used to calculate distance, record the relationship between sampling points and original points, and preserve the spatial characteristics of point cloud as much as possible. In order to solve the problem that RANSAC algorithm can not detect multiple planes at the same time, we mark the identified planes and apply RANSAC algorithm many times to meet the requirements. When a plane is found, mark the inner point of the plane (the point whose distance to the plane is less than the given threshold). When finding the next plane, the calculation continues only in the unmarked points. When the cut-off condition is reached (the identification result reaches the specified number of planes or the number of inner points of the last plane result is less than the set value), the algorithm terminates. In addition, we also use a variety of algorithms to optimize RANSAC.	

##### RANSAC Algorithm

The basic idea of RANSAC is to select multiple plane models by randomly selecting three points from the point cloud, and then calculate the number of interior points of each plane within the threshold. Finally, the plane with the most points is the best plane in the current point cloud.

##### Voxel Filter Sampling

The basic principle of voxel filter sampling is: split the point cloud space into cuboids or cubes of equal size. For each cube, select the point closest to the center of gravity of all points in the cuboid to represent the points of the whole cuboid.

##### Lo-RANSAC

Lo-RANSAC has many schemes to optimize RANSAC. In the process of iterative plane detecting, if the current optimal plane model appears, Lo-RANSAC can be considered. One method is to randomly select some points from the interior points of the current optimal plane model to fit the plane model again, calculate the interior points, iterate for a certain number of times, and select the best result as the improvement result.

##### Pruning

Pruning is to finish some unnecessary calculations in time when calculating the interior points of the plane model, which belongs to the optimization of details.

##### Total Least Square

In the plane model fitting part of RANSAC, when the Ordinary Least Square method is used to fit the plane, some special planes may not be fitted, such as plane $x = 0$. The calculation method of Total Least Square (TLS) can solve this problem. In addition, there are other plane fitting methods, and we are also prepared to select the final method through testing.

##### Early Termination of Iteration

Using the theories of probability and statistics, we can calculate the conditions for early termination of iteration. The principle is as follows:

* In a point cloud space "PC", the number of points is $N$, the number of interior points of the largest plane "PlaneMax" is $M$, and three points are randomly selected. The probability that all three points belong to the plane "PlaneMax" is $(\frac{M}{N})^3$.
* Each time three points are taken from the point cloud space to construct a plane, the probability that not "PlaneMax" is obtained for the continuous construction of $K$ planes is $(1 - (\frac{M}{N}) ^ 3) ^ K$, and the probability that at least one plane of $K$ planes is "PlaneMax" is $1 - (1 - (\frac{M}{N}) ^ 3) ^ K$.
* If we want to find the maximum plane correctly when the probability is not less than $P$, we can solve the equation $1 - (1 - (\frac{M}{N}) ^ 3) ^ K>P$ and get the result $K > \frac{log(1-P)}{log (1 - (\frac{M}{N}) ^ 3)}$ . So we only need to make the number of iterations not less than $\frac{log(1-P)}{log (1 - (\frac{M}{N}) ^ 3)}$.

- It can also be noted that even if the size of $M$ is not known at the beginning, the number of interior points of the maximum plane found so far saved in the iteration process $A$ will not be greater than $M $, so the algorithm only needs to make the number of iterations no less than $\frac{log(1-P)}{log (1 - (\frac{A}{N}) ^ 3)}$.

##### Normal Vector Constraint

In the actual test, we found that sampling will make the point cloud density uniform, resulting in the change of the order of plane recognition. Although it does not affect in multi plane recognition, it is considered that in practical applications, such as vehicle identification pavement, UAV identification wall and so on, only specific plane recognition is required. We use the vector constraint method to identify only the plane with small angle between the normal vector and the vector, and then we can detect the plane in a specific direction.

<br><br>

### Scheme Design

<img src="./images/framwork-EN.png" style="transform: scale(0.8);" />
