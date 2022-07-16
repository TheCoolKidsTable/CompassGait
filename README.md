# Passive Compass Gait

Model [1]:

![image](https://user-images.githubusercontent.com/41043317/157578449-d4a9d2d9-e5a4-4894-98c8-64f5dcb944f6.png)


https://user-images.githubusercontent.com/41043317/157576746-95a5be80-d314-4f3f-833f-526051747d68.mp4

# Instructions
Run `main.m`

Controller type is specified in `parameters.m`

`p.controller=1` : Passive (no input)

`p.controller=2` : Control via symmetry [1]

`p.controller=3` : Control via symmetry + passivity based control (PBC) [1]

`p.controller=4` : Feedback linearization [2]

`p.controller=5` : Energy pumping-and-damping [3]

Incline of slope can be changed by editing the `p.slope_change` within `parameters.m`

# References

[1] Passivity-Based Control of Bipedal Locomotion Regulating Walking by Exploiting Passive Gaits in 2-D and 3-D Bipeds ©2007 IEEEIEEE Robotics & Automation Magazine JUNE 200730

[2] Feedback Control Of Planar Biped With Regulable Step Length and Walking Speed 2010 IEEE Yong Hu, Gangfeng Yan, and Zhiyun Lin

[3] P. Arpenti, A. Donaire, F. Ruggiero and V. Lippiello, "Energy pumping-and-damping for gait robustification of underactuated planar biped robots within the hybrid zero dynamics framework," 2020 IEEE-RAS 20th International Conference on Humanoid Robots (Humanoids), 2021, pp. 415-421, doi: 10.1109/HUMANOIDS47582.2021.9555787.
