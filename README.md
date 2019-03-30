# eval-vislam

Toolkit for V-SLAM and VI-SLAM evaluation.

For more information, please refer to our [project website](http://www.zjucvg.net/eval-vislam/).

## Prerequisites

- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

## Toolkit Usage

### Accuracy (APE, RPE, ARE, RRE, Completeness)

```docopt
Usage:
  ./accuracy <groundtruth> <input> <fix scale>

Arguments:
  <groundtruth>   Path to sequence folder, e.g. ~/VISLAM-Dataset/A0.
  <input>         SLAM camera trajectory file in TUM format(timestamp[s] px py pz qx qy qz qw).
  <fix scale>     Set to 1 for VI-SLAM, set to 0 for V-SLAM.
```

### Initialization Scale Error and Time

```docopt
Usage:
  ./initialization <groundtruth> <input> <has inertial>

Arguments:
  <groundtruth>   Path to sequence folder, e.g. ~/VISLAM-Dataset/A0.
  <input>         SLAM camera trajectory file in TUM format(timestamp[s] px py pz qx qy qz qw).
  <has inertial>  Set to 1 for VI-SLAM, set to 0 for V-SLAM.
```

### Robustness

```docopt
Usage:
  ./robustness <groundtruth> <input> <fix scale>

Arguments:
  <groundtruth>   Path to sequence folder, e.g. ~/VISLAM-Dataset/A0.
  <input>         SLAM camera trajectory file in TUM format(timestamp[s] px py pz qx qy qz qw).
  <fix scale>     Set to 1 for VI-SLAM, set to 0 for V-SLAM.
```

### Relocalization Time

```docopt
Usage:
  relocalization <groundtruth> <input> <has inertial>

Arguments:
  <groundtruth>   Path to sequence folder, e.g. ~/VISLAM-Dataset/A0.
  <input>         SLAM camera trajectory file in TUM format(timestamp[s] px py pz qx qy qz qw).
  <has inertial>  Set to 1 for VI-SLAM, set to 0 for V-SLAM.
```
