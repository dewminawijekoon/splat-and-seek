# Splat-and-Seek

**A few-watt Gaussian Splatting localizer for robots in radio-silent sites, on PolarFire SoC.**

A robot that inspects a GPS-denied site, such as a nuclear plant or a mine, has to know where it
is without a radio link. Splat-and-Seek surveys the site once with a camera and turns the video
into a 3D Gaussian Splatting map. On the robot, a custom FPGA engine renders what the camera
should see from hundreds of guessed positions and scores each guess against the real camera
frame, while a real-time RISC-V core runs the particle filter over those scores.

Entry for the **PolarFire FPGA Design Contest 2026, Track 2 (PolarFire SoC Icicle Kit)**.
Currently at the proposal stage; the build phase runs from 30 November 2026 to 27 March 2027.

## Contents

- [docs/proposal/](docs/proposal/): the proposal, with the system architecture and mission
  flowchart as an Excalidraw drawing and a rendered image.
- [docs/project/plan.md](docs/project/plan.md): the project plan: phases, checkpoints, tasks and
  risks.

## Authors

[dewminawijekoon](https://github.com/dewminawijekoon) and [YutharsanS](https://github.com/YutharsanS),
University of Moratuwa.
