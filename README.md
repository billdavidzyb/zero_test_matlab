# zero_test_matlab

## Intro
A prototype kinematic calibration scripts for zero point error calibration of 6 axis manipulator.

## Problem
Zero point error comes from the drift of joints. Namely, the operator commands all joints to return to their own default positions, while actually they might reach some other positions.

## Explanation
To this end, a set of joint positions must be recorded with tool center point (tcp) pointed on the same point. (Ideally, all poses recorded share the same positions (x, y, z) and different poses (alpha, beta, gamma)). For accuracy, 15 or more poses should be sampled.
