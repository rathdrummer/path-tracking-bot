# path-tracking-bot

Project for AI class in Robotics and Control - Umea University

So far, a lot of features have been implemented, with no proper tracking algorithms as of yet.
The script can read the adequate JSON files as well as connect to an MRDS server to send commands and receive data.
The point of the extensive functionality implementation was to make the algorithms a breeze to write and also to read afterwards.

Notable implemented features:
- Robot class: allows for easy update and treatment of coordinates. Calling updateAttributes() updates the Robot's stored coordinates i.e. xyz position and heading. Other object methods include getDistance(x,y), getBearing(x,y)..


  Path tracking algorithm to be implemented : Pure Pursuit

A "carrot point" is placed at a short distance ahead of the bot. The bot heads towards this point. Once close enough, the carrot point is then shifted further along the path.
This tracking method alone is known as "carrot on a stick" and leads to erratic oscillations around the path. To counter this, instead of heading directly towards the carrot point, an arc is calculated between the robot and the point, based on the robot's heading. The robot follows the arc towards the point. This improved method is Pure Pursuit. It will be more stable than Carrot on a Stick, but possibly slower?

Possible algorithm draft:

1. Check distance from carrot point. If closer than given value, shift carrot point up (one point? more?).
2. Calculate necessary angular/linear speed to follow circular arc to carrot point.

Still to be implemented:

- Circular arc calculations: calculate a constant angular/linear speed necessary to reach nearby carrot point. Maybe have linear speed inversely proportional to angular speed to avoid robot toppling? Testing required!!
- Timing code: Once the bot sets off on the path, a timer is started. Once it reaches the final point, the time is displayed on standard output.
- Laser detection: If the bot detects an obstacle, go closer to path to move around. Maybe decrease look-ahead distance? To discuss in later stages. 
