# arduino_robot
At our university, there were some after-school introductory courses to Arduino and electronics in general, so I decided to build some kind of moving robot.
Originally, it was meant to be a sumo robot, but as I didn't know much about electronics at all I made the wrong choice of going with continuous rotation servos for driving, which don't have much torque at all. What they do offer is quite precise control of rotation speed of each motor, so you can make it move in many cool ways quite easily.

I first did lots of tests to get used to the programming and the different components (H bridge motor control, servo control, ultrasonic and IR distance sensors, bluetooth...).

Then I was too ambitious and tried to put all these test files together in one big file, thinking it would magically work, which it of course didn't. I spent lots of time trying to debug this mess, but it was all in vain.

I ended up completely rewriting the code from scratch, scrapping lots of superfluous things in the process, and the result can be found in the all_done folder
