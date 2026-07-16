INTRODUCTION:
# E-Team Squadra Corse: EKF-SLAM Node Documentation

Authors:
[Giulio Ferrulli](https://github.com/GiulioFerr01)<g.ferrulli@student.unipi.it>

This documentation provides an in-depth overview of the Extended Kalman Filter (EKF) based Simultaneous Localization and Mapping (SLAM) node. The node is designed to estimate the vehicle's pose and build a global map of the track cones in real-time, handling the non-linear dynamics of a Formula Student racecar.

## Table of Contents
0. [Installation Prerequisites](0-installation-prerequisites.md)
1. [Node Interface](#node-interface)
2. [EKF Core Logic & Architecture](#ekf-core-logic--architecture)
3. [Latency Reduction & Optimization](#latency-reduction--optimization)
4. [Current Test Results](#current-test-results)

---