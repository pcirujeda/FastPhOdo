# FastPhOdo

**WIP**

A very simple visual odometry estimator using essential matrix estimation with FAST descriptors in monocular image sequences.

Testing on the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) so far.

![](KITTI_dataset_sequences/2011_09_26_drive_0005_sync.gif)

![](KITTI_dataset_sequences/odometry_trajectory.gif)

### ToDo
- Error checking
- Include testing (class, unit, dataset groundtruth)
- Add detection of outlier motions for intra-frame variability motion compensation
- Include standalone pinhole model camera calibration app
- Try scale estimation methods using camera height relative to ground-plane
- Create app for real-time video/camera trajectory estimation
- Provide better verbose visualization

### Dependencies
[OpenCV](https://opencv.org/)
[Boost](https://www.boost.org/) for file and argument parsing

Tested on Mac OsX only

### Build'n run
Use the provided cmake file:

```
mkdir build && cd ./build
cmake ..
make
```

Call the provided test executable with any acquired sequence and its acquisition camera parameters (a sequence from the KITTI dataset and its available camera calibration parameters are included for out of the box testing):

> ./phodo --help

```
./phodo --sequence-directory ../KITTI_dataset_sequences/2011_09_26_drive_0005_sync/image_00/data/ --focal-length 718.8560 --principal-point-x 608.1928 --principal-point-y 185.2157
```

### References
ToDo

For the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) sequence included:
> A. Geiger, P. Lenz, C. Stiller and R. Urtasun, *"Vision meets Robotics: The KITTI Dataset"*, in International Journal of Robotics Research, 2013.
