import wpilib
import commands2
from subsystems.swervedrive import SwerveDrive

class Robot(commands2.TimedCommandRobot):
    swerve_drive: SwerveDrive

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.swerve_drive = SwerveDrive()

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        self.container.disablePIDSubsystems()

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass
        # self.autonomousCommand = None

        # if self.autonomousCommand:
        #     self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass
        # if self.autonomousCommand:
        #     self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(Robot)