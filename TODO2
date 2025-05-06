## simple solution to use of nonreentrant sprintf
wrap all sprintf in critical section (disable interrupts = disable context switching)
https://www.freertos.org/taskENTER_CRITICAL_taskEXIT_CRITICAL.html

## Enable FreeRTOS debugging in stm32CubeIDE

## in LIP_task_communication.c 
sprintf is used with %f which uses non-reentrant malloc
spritnf used with decimal output is not calling malloc anywhere
info source: https://www.nadler.com/embedded/newlibAndFreeRTOS.html

solution
    split float into decimal and fractional parts
    use modf from math.h

## newlib / newlib nano isnot reentrant
Functions like printf may use non reentrant mem alloc functions like malloc.
More info:
    https://mcuoneclipse.com/2017/07/02/using-freertos-with-newlib-and-newlib-nano/ 
    https://www.nadler.com/embedded/newlibAndFreeRTOS.html
    https://community.st.com/t5/stm32cubemx-mcus/bug-cubemx-freertos-projects-corrupt-memory/m-p/267070

    https://mcuoneclipse.com/2020/11/15/steps-to-use-freertos-with-newlib-reentrant-memory-allocation/

Couldn't disable dynamic mem allocation becouse FreeRTOS_CLI uses it internally.

Solution
    Write new CLI functionality not based on freeRTOS cli
    then, dynamic mem alloc can be safely disabled in FreeRTOSConfig.h

Problem
    FreeRTOS debugging features use FreeRTOS dynamic mem alloc api (which is reentrant but it's still dynamic mem alloc)

## States
States should be changed, when task, corresponding to a given state, is resumed. 
Now, state is changed outside of its corresponding task - this creates bugs, 
for eg. DPC can be running but state is DEFAULT.

## Bounceoff
At this moment, bounce off functionality is very poor. 
It just changes the controller cart position setpoint and sometimes setpoint 
change is so slow (because of it is low-pass filtered) that there is no bounce off at all,
cart just hits the max or min limit switch and turns of the voltage


