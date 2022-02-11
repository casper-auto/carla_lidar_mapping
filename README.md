# Carla Lidar Mapping

Mapping with Ground-truth localization.

## Carla version

The demo was tested with Carla 0.9.11, though the version does not matter for the mapping itself. Feel free to test it with other Carla versions.

## Motivation

**Be short:** To purely observe how mapping works, without coupling with localization problems.

Simultaneous Localization and Mapping (SLAM) is treated a complicated problem partially because we have to deal with two problems together and they are mutually dependent, which is like a "chicken and egg" problem.

In fact, the two problems are not in the same scale considering the complexity. Why not start from the easier one, which is mapping? In reality, when we say mapping, it is always a different wording of SLAM, or LAM if we don't care about the real-time processing. With a simulator, it becomes possible to do mapping with perfect localization. In that case, we can find mapping is more or less just a problem of coordinate transformation.

## PCL Viewer

Install:

```
sudo apt install pcl-tools
```

Use:

```
pcl_viewer my_map.pcd
```
