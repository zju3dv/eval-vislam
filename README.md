# eval-vislam

Toolkit for VSLAM and VISLAM evaluation.

For more information, please refer to our [project website](http://www.zjucvg.net/eval-vislam/).

## License

This project is released under the Apache 2.0 license.

## Prerequisites

- **A modern compiler, i.e., at least gcc 7.1 on Linux.**
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

## ISMAR 2019 SLAM Challenge Scoring ToolKit

```docopt
Usage:
  python3 ismar-score.py --round <round> --is_vislam <is_vislam> --trajectory_base_dir <trajectory_base_dir> --gt_base_dir <gt_base_dir>

Arguments:
  <round>                    SLAM benchmark number of rounds, e.g. 5.
  <is_vislam>                Set to 1 for VISLAM, set to 0 for VSLAM.
  <trajectory_base_dir>      SLAM camera trajectory file folders(e.g. ~/MY-SLAM/trajectories/). We support TUM format(timestamp[s] px py pz qx qy qz qw) files.
  <gt_base_dir>              Path to groundtruth folder, e.g. ~/ISMAR-Dataset/train.
```

## Toolkit Usage

### Accuracy (APE, RPE, ARE, RRE, Completeness)

```docopt
Usage:
  ./accuracy <groundtruth> <input> <fix scale>

Arguments:
  <groundtruth>    Path to sequence folder, e.g. ~/VISLAM-Dataset/A0.
  <input>          SLAM camera trajectory file in TUM format(timestamp[s] px py pz qx qy qz qw).
  <fix scale>      Set to 1 for VISLAM, set to 0 for VSLAM.
```

### Initialization Scale Error and Time

```docopt
Usage:
  ./initialization <groundtruth> <input> <has inertial>

Arguments:
  <groundtruth>    Path to sequence folder, e.g. ~/VISLAM-Dataset/A0.
  <input>          SLAM camera trajectory file in TUM format(timestamp[s] px py pz qx qy qz qw).
  <has inertial>   Set to 1 for VISLAM, set to 0 for VSLAM.
```

### Robustness

```docopt
Usage:
  ./robustness <groundtruth> <input> <fix scale>

Arguments:
  <groundtruth>    Path to sequence folder, e.g. ~/VISLAM-Dataset/A0.
  <input>          SLAM camera trajectory file in TUM format(timestamp[s] px py pz qx qy qz qw).
  <fix scale>      Set to 1 for VISLAM, set to 0 for VSLAM.
```

### Relocalization Time

```docopt
Usage:
  relocalization <groundtruth> <input> <has inertial> <jump detection>

Arguments:
  <groundtruth>    Path to sequence folder, e.g. ~/VISLAM-Dataset/A0.
  <input>          SLAM camera trajectory file in TUM format(timestamp[s] px py pz qx qy qz qw).
  <has inertial>   Set to 1 for VISLAM, set to 0 for VSLAM.
  <jump detection> Sensitivity to detect jump when relocalization happened (Default 0.05).
```

## Citation

If you are using our codebase or dataset for research, please cite the following publication:

```bibtex
@article{
  title   = {Survey and Evaluation of Monocular Visual-Inertial SLAM Algorithms for Augmented Reality},
  author  = {Jinyu Li, Bangbang Yang, Danpeng Chen, Nan Wang, Guofeng Zhang*, Hujun Bao*},
  journal = {Journal of Virtual Reality & Intelligent Hardware},
  year    = {2019},
  volume  = {1},
  number  = {4},
  pages   = {386-410},
  url     = {http://www.vr-ih.com/vrih/html/EN/10.3724/SP.J.2096-5796.2018.0011/article.html},
  doi     = {10.3724/SP.J.2096-5796.2018.0011}
}
```
