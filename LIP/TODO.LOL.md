## States
States should be changed, when task, corresponding to a given state, is resumed. 
Now, state is changed outside of its corresponding task - this creates bugs, 
for eg. DPC can be running but state is DEFAULT.

## Bounceoff
At this moment, bounce off functionality is very poor. 
It just changes the controller cart position setpoint and sometimes setpoint 
change is so slow (because of it is low-pass filtered) that there is no bounce off at all,
cart just hits the max or min limit switch and turns of the voltage
 