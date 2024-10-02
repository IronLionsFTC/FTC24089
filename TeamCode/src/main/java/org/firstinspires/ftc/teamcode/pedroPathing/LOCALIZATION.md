## Overview
This is the localization system developed for the Pedro Pathing path follower. These localizers use
the pose exponential method of localization. It's basically a way of turning movements from the
robot's coordinate frame to the global coordinate frame. If you're interested in reading more about
it, then check out pages 177 - 183 of [Controls Engineering in the FIRST Robotics Competition](https://file.tavsys.net/control/controls-engineering-in-frc.pdf)
by Tyler Veness.

## Setting Your Localizer
Go to line `70` in the `PoseUpdater` class, and replace the `new ThreeWheelLocalizer(hardwareMap)`
with the localizer that applies to you:
* If you're using three wheel odometry, put `new ThreeWheelLocalizer(hardwareMap)`
* If you're using three wheel odometry with the IMU, put `new ThreeWheelIMULocalizer(hardwareMap)`

## Tuning

# Three Wheel Localizer
### Pre-Tuning Configuration
* First, you'll need three odometry wheels connected to motor encoder ports on a hub.
* Then, go to `ThreeWheelLocalizer.java`. First, in the constructor, enter in the positions of your
  tracking wheels relative to the center of the wheels of the robot. The positions are in inches, so
  convert measurements accordingly. Use the comment above the class declaration to help you with the
  coordinates.
* Next, go to where it tells you to replace the current statements with your encoder ports in the constructor.
  Replace the `deviceName` parameter with the name of the port that the encoder is connected to. The
  variable names correspond to which tracking wheel should be connected.
* Then, reverse the direction of any encoders so that the forward encoders tick up when the robot
  is moving forward and the strafe encoder ticks up when the robot moves right.
### Tuning
* First, start with the `Turn Localizer Tuner`. You'll want to position your robot to be facing
  in a direction you can easily find again, like lining up an edge of the robot against a field tile edge.
  By default, you should spin the robot for one rotation going counterclockwise. Once you've spun
  exactly that one rotation, or whatever you set that value to, then the turn multiplier will be shown
  as the second number shown. The first number is how far the robot thinks you've spun, and the second
  number is the multiplier you need to have to scale your current readings to your goal of one rotation,
  or the custom set angle. Feel free to run a few more tests and average the results. Once you have
  this multiplier, then replace `TURN_TICKS_TO_RADIANS` in the localizer with your multiplier. Make sure
  you replace the number, not add on or multiply it to the previous number. The tuner takes into
  account your current multiplier.
* Next, go on to `Forward Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches forward. Once you've pushed that far, or whatever
  you set that value to, then the forward multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `FORWARD_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Finally, go to `Lateral Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches to the right. Once you've pushed that far, or whatever
  you set that value to, then the lateral multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `STRAFE_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Once you're done with all this, your localizer should be tuned. To test it out, you can go to
  `Localization Test` and push around or drive around your robot. Go to [FTC Dashboard](http://192.168.43.1:8080/dash)
  and on the top right, switch the drop down from the default view to the field view. Then, on the bottom
  left corner, you should see a field and the robot being drawn on the field. You can then move your
  robot around and see if the movements look accurate on FTC Dashboard. If they don't, then you'll
  want to re-run some of the previous steps. Otherwise, congrats on tuning your localizer!

# Three Wheel Localizer with IMU
* First, you'll need three odometry wheels connected to motor encoder ports on a hub.
* Then, go to `ThreeWheelIMULocalizer.java`. First, in the constructor, enter in the positions of your
  tracking wheels relative to the center of the wheels of the robot. The positions are in inches, so
  convert measurements accordingly. Use the comment above the class declaration to help you with the
  coordinates.
* Next, go to where it tells you to replace the current statements with your encoder ports in the constructor.
  Replace the `deviceName` parameter with the name of the port that the encoder is connected to. The
  variable names correspond to which tracking wheel should be connected.
* After that, go to the instantiation of the IMU and change the orientation of the IMU to match that
  of your robot's.
* Then, reverse the direction of any encoders so that the forward encoders tick up when the robot
  is moving forward and the strafe encoder ticks up when the robot moves right.
* Although heading localization is done mostly through the IMU, the tracking wheels are still used for
  small angle adjustments for better stability. So, you will still need to tune your turning multiplier.
* First, start with the `Turn Localizer Tuner`. Before doing any tuning, go to FTC Dashboard and find
  the `ThreeWheelIMULocalizer` dropdown and deselect `useIMU`. You'll want to position your robot to be facing
  in a direction you can easily find again, like lining up an edge of the robot against a field tile edge.
  By default, you should spin the robot for one rotation going counterclockwise. Once you've spun
  exactly that one rotation, or whatever you set that value to, then the turn multiplier will be shown
  as the second number shown. The first number is how far the robot thinks you've spun, and the second
  number is the multiplier you need to have to scale your current readings to your goal of one rotation,
  or the custom set angle. Feel free to run a few more tests and average the results. Once you have
  this multiplier, then replace `TURN_TICKS_TO_RADIANS` in the localizer with your multiplier. Make sure
  you replace the number, not add on or multiply it to the previous number. The tuner takes into
  account your current multiplier.
* Next, go on to `Forward Localizer Tuner`. You should re-enable `useIMU` at this time. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches forward. Once you've pushed that far, or whatever
  you set that value to, then the forward multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `FORWARD_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Finally, go to `Lateral Localizer Tuner`. `useIMU` should be enabled for this step. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches to the right. Once you've pushed that far, or whatever
  you set that value to, then the lateral multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `STRAFE_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Once you're done with all this, your localizer should be tuned. Make sure that `useIMU` is turned back on. To test it out, you can go to
  `Localization Test` and push around or drive around your robot. Go to [FTC Dashboard](http://192.168.43.1:8080/dash)
  and on the top right, switch the drop down from the default view to the field view. Then, on the bottom
  left corner, you should see a field and the robot being drawn on the field. You can then move your
  robot around and see if the movements look accurate on FTC Dashboard. If they don't, then you'll
  want to re-run some of the previous steps. Otherwise, congrats on tuning your localizer!