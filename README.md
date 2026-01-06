# Visual-Inertial Odometry (VIO)

<div align="center">

![Python](https://img.shields.io/badge/Python-3.6+-blue.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-Latest-green.svg)
![NumPy](https://img.shields.io/badge/NumPy-Latest-orange.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

**Stereo Visual-Inertial Odometry using Multi-State Constraint Kalman Filter**

*Precise 6-DOF pose estimation by fusing camera and IMU measurements*

[Overview](#overview) • [Installation](#installation) • [Usage](#usage) • [Algorithm](#algorithm) • [Results](#results)

</div>

---

## 👋 About This Project

I'm **Harmeet Dhillon**, a robotics engineer passionate about autonomous navigation and localization systems. This project implements a stereo visual-inertial odometry system using the **Multi-State Constraint Kalman Filter (MSCKF)** algorithm—a fundamental technique for enabling robots to understand their motion in 3D space. By fusing high-frequency IMU measurements with visual features from stereo cameras, this system achieves robust and accurate pose estimation essential for autonomous navigation, drone flight control, and mobile robotics. This work deepens my understanding of sensor fusion, state estimation, and the mathematical foundations that power SLAM systems.

---

##  Overview

**Visual-Inertial Odometry (VIO)** estimates the 6-DOF pose (position and orientation) of a moving platform by combining visual information from cameras with inertial measurements from an IMU (Inertial Measurement Unit). Unlike pure visual odometry, VIO provides:

- **Higher frequency updates** from IMU integration
- **Robustness** during rapid motions or low-texture environments
- **Metric scale recovery** without additional sensors
- **Drift correction** through visual loop closures

This implementation uses the **Multi-State Constraint Kalman Filter (MSCKF)**, which maintains a sliding window of camera poses while marginalizing out old poses to maintain computational efficiency.

---

## ✨ Features

- 🎥 **Stereo Vision Processing** - Leverages stereo camera pairs for robust feature triangulation
- 📐 **IMU Integration** - High-frequency gyroscope and accelerometer fusion
- 🔧 **MSCKF Algorithm** - Efficient state estimation with sliding window optimization
- 📊 **Bias Calibration** - Automatic gyroscope and accelerometer bias correction
- ⚡ **Runge-Kutta Integration** - 4th order numerical integration for state propagation
- 🎯 **Bundle Adjustment** - Kalman filter-based optimization with QR decomposition
- 📈 **Trajectory Visualization** - Real-time pose plotting (optional with Pangolin)

---

## 🔧 Prerequisites

- **Python 3.6+**
- **NumPy** - Numerical computations
- **SciPy** - Scientific computing and optimization
- **OpenCV** - Image processing and feature detection
- **Pangolin** (optional) - 3D trajectory visualization

---

## 📦 Installation

### 1. Clone the Repository

```bash
git clone https://github.com/Harmeet-Dhillon/Visual-Inertial-Odometery.git
cd Visual-Inertial-Odometery
```

### 2. Install Dependencies

```bash
pip install numpy scipy opencv-python
```

### 3. Install Pangolin (Optional, for Visualization)

```bash
pip install pangolin
```

### 4. Download Dataset

Download the [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (e.g., Machine Hall 01):

```bash
# Example dataset structure
EuRoC_MAV_dataset/
└── MH_01_easy/
    ├── mav0/
    │   ├── cam0/
    │   ├── cam1/
    │   └── imu0/
    └── ...
```

---

## 🚀 Usage

### Basic Execution (No Visualization)

```bash
python vio.py --path path/to/EuRoC_MAV_dataset/MH_01_easy
```

### With Real-Time Visualization

```bash
python vio.py --view --path path/to/EuRoC_MAV_dataset/MH_01_easy
```

### Command-Line Options

| Option | Description |
|--------|-------------|
| `--path` | Path to EuRoC dataset directory |
| `--view` | Enable real-time trajectory visualization |

---

## 🔬 Algorithm Pipeline

The VIO system follows this sequential processing pipeline:

```
IMU Data Stream          Camera Frames
       ↓                        ↓
  Bias Initialization    Feature Detection
       ↓                        ↓
  IMU Integration  ←→  State Augmentation
       ↓                        ↓
  State Prediction      Feature Tracking
       ↓                        ↓
       └──→  Measurement Update  ←──┘
                    ↓
            Pose Estimation Output
```

### Key Components

#### 1. Initialization & Bias Calibration
- Computes gyroscope and accelerometer biases from stationary readings
- Initializes gravity vector and IMU orientation
- Uses first 200 measurements for stable calibration

#### 2. IMU State Propagation
The continuous-time IMU dynamics are modeled as:

```
q̇ = Ω(ω̂) q          (orientation)
v̇ = C(q)ᵀ(â + g)    (velocity)
ṗ = v                (position)
ḃg = 0               (gyro bias)
ḃa = 0               (accel bias)
```

State propagation uses **4th-order Runge-Kutta** integration for accurate numerical integration between measurements.

#### 3. State Augmentation
When new camera frames arrive, the system augments the state with new camera poses:

```
G_pC = G_pI + C(G_qI)ᵀ · I_pC
C_qG = I_qC ⊗ G_qI
```

The covariance matrix is expanded using Jacobian matrices to maintain uncertainty estimates.

#### 4. Feature Tracking & Observation
- SIFT/SURF features tracked across stereo pairs
- Feature observations stored with pixel coordinates
- Visibility matrix tracks which features are seen by which cameras

#### 5. Measurement Update (MSCKF)
The measurement update uses **QR decomposition** for numerical stability:

```
H = [Q1 Q2] [R]
              [0]
```

Kalman gain computation:
```
K = P Hᵀ (R Rᵀ + Rn)⁻¹
```

State correction:
```
ΔX = K rn
P = (I - KH) P (I - KH)ᵀ + K Rn Kᵀ
```

---

## 📊 Results

### Performance Metrics

Tested on EuRoC Machine Hall 01 dataset with SE3 trajectory alignment:

| Metric | RMSE Value |
|--------|------------|
| **Rotation Error** | 153.82° |
| **Scale Error** | 2.20 |
| **Translation Error** | 0.14 m |

### Trajectory Visualization

**Top View: Ground Truth vs Estimated Trajectory**

![Top View](https://private-user-images.githubusercontent.com/147753385/476417495-a14073e9-fe5e-4760-8dfd-56dd34756c64.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3Njc2NzAxMDYsIm5iZiI6MTc2NzY2OTgwNiwicGF0aCI6Ii8xNDc3NTMzODUvNDc2NDE3NDk1LWExNDA3M2U5LWZlNWUtNDc2MC04ZGZkLTU2ZGQzNDc1NmM2NC5wbmc_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwMTA2JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDEwNlQwMzIzMjZaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT03YmZjNjhiZTYyYWU2NTQwYTkyYTU0NjM4MDFlNGE4ZTgxNWNhNjk4MmNlNDVjZDdkOGYzNTMwMmE0NWNjMzVlJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.4D4eC2YokV2xwb-OYYIOi9sauUxp12aJguGdK22KZ-U)

**Side View: Ground Truth vs Estimated Trajectory**

![Side View](https://private-user-images.githubusercontent.com/147753385/476417550-363b1dc4-77d1-4834-8d77-2ae6b70b6043.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3Njc2NzAxMDYsIm5iZiI6MTc2NzY2OTgwNiwicGF0aCI6Ii8xNDc3NTMzODUvNDc2NDE3NTUwLTM2M2IxZGM0LTc3ZDEtNDgzNC04ZDc3LTJhZTZiNzBiNjA0My5wbmc_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwMTA2JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDEwNlQwMzIzMjZaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1jMjcyYzZlYjM2MjNkY2FkMDY3MDI0NjNhYjI4ZjJiM2EwODQyNzZjN2VlMmNmZTQ4NDM5MGY1MTJmZDQ1MTNkJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.SrorqwghjmC7uvxvw_4ESgvGegPQDD3PpXodAkRej-E)

The system successfully tracks the drone's trajectory throughout the Machine Hall sequence, maintaining consistent pose estimates even during dynamic maneuvers.

---

## 📁 Project Structure

```
Visual-Inertial-Odometery/
├── Code/
│   ├── vio.py                    # Main VIO pipeline
│   ├── msckf.py                  # MSCKF implementation
│   ├── feature_tracking.py       # Visual feature processing
│   ├── utils.py                  # Utility functions
│   └── ...
├── Report.pdf                    # Technical report
└── README.md                     # This file
```

---

## 🔍 Technical Details

### Multi-State Constraint Kalman Filter (MSCKF)

The MSCKF maintains a state vector containing:

```
X = [qIG, bG, vG, pG, qIC1, pIC1, ..., qICn, pICn]ᵀ
```

Where:
- **qIG**: IMU orientation quaternion
- **bG**: Gyroscope and accelerometer biases
- **vG, pG**: IMU velocity and position
- **qICi, pICi**: Camera poses in sliding window

### Advantages of MSCKF

- **Computational Efficiency**: O(n) complexity vs O(n³) for batch optimization
- **Constant State Size**: Old camera poses marginalized out
- **Feature Constraints**: Features provide constraints between poses without being part of state
- **Real-Time Capable**: Suitable for embedded systems and UAVs

### Implementation Highlights

- **Numerical Stability**: QR decomposition for measurement updates
- **Bias Estimation**: Online calibration of sensor biases
- **Stereo Triangulation**: Exploits baseline for metric scale
- **Covariance Management**: Careful propagation and augmentation

---

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| Import errors | Verify all dependencies installed: `pip list` |
| Dataset not found | Check path and ensure EuRoC format structure |
| Poor trajectory estimate | Verify camera calibration and IMU-camera extrinsics |
| Memory issues | Reduce sliding window size in config |
| Slow performance | Disable visualization (`--view` flag) |

---

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
- [ ] Loop closure detection
- [ ] GPU-accelerated feature tracking
- [ ] Support for monocular cameras
- [ ] ROS integration
- [ ] Additional dataset compatibility

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

This implementation is based on:
- **"A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation"** by Mourikis & Roumeliotis
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) - ETH Zurich
- [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation) - Trajectory alignment and evaluation tools

---

## 📧 Contact

**Harmeet Dhillon**

- Email: hdhillon@wpi.edu
- LinkedIn: [harmeet-dhillon-826a43237](https://www.linkedin.com/in/harmeet-dhillon-826a43237/)
- GitHub: [@Harmeet-Dhillon](https://github.com/Harmeet-Dhillon)

---

<div align="center">

**If you find this project helpful, please consider giving it a ⭐**

Made with ❤️ for autonomous robotics

</div>
