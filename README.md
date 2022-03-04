# FTC4890 (Hive Voltage 2.0)
A repository of Team 4890's FTC code!

## Welcome to all of FTC!
Here you can find all of the code from Team 4890!
This repository will primarily be used by members of 4890 so that the code can be shared between team members, though other teams are welcome to utilize the code in better understanding Java and how the team programs the robot!

# Release Information

# Version 3.5 (March 4, 2022 - NJ STATES)

## Additions

- Implemented RoadRunner in Autonomous (With Encoders)

# Version 3.0 (December 20, 2021)

## Additions

- Updated TeleOp Code & Autonomous (No Encoders) for this year's season.

## Known Bugs *(to be fixed)*

- Encoder Autonomous Incomplete

# Version 2.0 (February 7, 2021)

## Additions

- Drivetrain now supports arcade controls (for Driver 1, left analog stick controls movement while right analog stick controls rotation).

## Removals

- Power & time autonomous scrapped, beginning work on Road Runner for our autonomous now.*

## Known Bugs *(to be fixed)*

* Robot still needs to be tuned for Road Runner before beginning actual work on the autonomous itself.

# Version 1.3.3 (December 9, 2020)

## Additions

- Completed autonomous for picking up both wobble goals & parking in all three possible ring setups. Thought it is not entirely consistent mainly because of robot placement & the drivetrain motors.

## Removals

- Intake is no longer an off or on toggle, instead three buttons are used to intake, reverse the intake, or to turn it off.

## Known Bugs *(to be fixed)*

- Outtake speeds are only theoretical, have not been tested yet.

# Version 1.3.2 (December 3, 2020)

## Additions

- Phone flashlight support for use during autonomous.

## Bug Fixes

- Straight method in autonomous is reversed as previously a positive power would make the robot move backwards instead of foward.
- Ring detection if & else statements for the robot moved into the opModeIsActive while loop, and modified so that it will only briefly check for the amount of rings once the autonomous mode starts.*

## Known Bugs *(to be fixed)*

* Autonomous code has not been tested or completed, currently it is only for testing if the robot will complete different actions based on number of rings.
* Outtake speeds are only theoretical, have not been tested yet.

# Version 1.3.1 (November 22, 2020)

## Additions

- Added support for another servo on the launcher as two are now needed again.

## Bug Fixes

- Arm motor and outtake motor reversed in the code as they were turning the wrong way.
- For the new claw design, the claw servo now opens up all the way when it releases. Originally only opened up halfway because of the original claw design.

## Known Bugs *(to be fixed)*

* Autonomous code has not been tested or completed, currently it is only for testing if the robot will complete different actions based on number of rings.
* Outtake speeds are only theoretical, have not been tested yet.

# Version 1.3 (November 22, 2020)

## Additions

- Began work on the Autonomous with EasyOpenCV (added support for it).*

## Removals

- Removed the right outtake as the outtake will only use the left motor now.
- Redundant code removed.

## Known Bugs *(to be fixed)*

* *Code has not been tested or completed, currently it is only for testing if the robot will complete different actions based on number of rings.

# Version 1.2 (November 15, 2020)

## Additions

- Luancher functionality in teleop completed and working properly.
- Claw system redesigned for 180 servo, completed and working properly.

## Bug Fixes

- Arm motor slowed down to 30% of normal speed, max speed resulted in an uncontrollable arm.

## Known Bugs *(to be fixed)*

* Outtake speeds are theoretical, have not been tested yet for actual use in firing rings into the goals.

# Version 1.1 (November 14, 2020)

## Additions

* Added support for the following:
    - Wobble goal claw system.*
    - Outtake system with adjustable speed.
    - Ring launcher positioning system.**
    - Ring pusher.

## Bug Fixes

* Fixed the toggle functions for the intake.
* Removed redundant code such as certain methods.

## Known Bugs *(to be fixed)*

* *Servo values of ring launcher incorrect, needs to be tested further so that the launcher is angled correctly. Positioning functions also not working and needs to be worked on as well.
* **Claw system of wobble goal may or may not be functioning correctly, servo for it needs to be replaced.

# Version 1.0 (October 29, 2020)

* First release ever, woohoo!
* First revision of Teleop code added.
