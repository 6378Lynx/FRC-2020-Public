/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.control.Spin;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final ColorSubsystem colorSubsystem = new ColorSubsystem();
  XboxController xboxController = new XboxController(Constants.OIConstants.driverControllerPort);
  XboxController operatorController = new XboxController(Constants.OIConstants.operatorControllerPort);
  Joystick controller = new Joystick(Constants.OIConstants.driverControllerPort);
  Joystick operator = new Joystick(Constants.OIConstants.operatorControllerPort);

  Button  driverA_Button = new JoystickButton(controller, Constants.OIConstants.DRIVER_A_Button),
          driverB_Button = new JoystickButton(controller, Constants.OIConstants.DRIVER_B_Button),
          driverY_Button = new JoystickButton(controller, Constants.OIConstants.DRIVER_Y_Button),
          driverX_Button = new JoystickButton(controller, Constants.OIConstants.DRIVER_X_Button),
          driverLeftBumper  = new JoystickButton(controller, Constants.OIConstants.DRIVER_leftBumper),
          //driverStartButton = new JoystickButton(joystick, 10),

          //OPERATOR
          operatorStart_Button = new JoystickButton(operator, Constants.OIConstants.OPERATOR_startButton),
          operatorA_Button     = new JoystickButton(operator, Constants.OIConstants.OPERATOR_A_Button),
          operatorB_Button     = new JoystickButton(operator, Constants.OIConstants.OPERATOR_B_Button),
          operatorY_Button     = new JoystickButton(operator, Constants.OIConstants.OPERATOR_Y_Button),
          operatorX_Button     = new JoystickButton(operator, Constants.OIConstants.OPERATOR_X_Button),
          operatorLeftBumper  = new JoystickButton(operator, Constants.OIConstants.OPERATOR_leftBumper),
          operatorRightBumper  = new JoystickButton(operator, Constants.OIConstants.OPERATOR_rightBumper);




  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    robotDrive.setDefaultCommand(
            new RunCommand(
                    () -> robotDrive.curvatureDrive(xboxController.getY(GenericHID.Hand.kLeft), xboxController.getX(GenericHID.Hand.kRight), xboxController.getBumper(GenericHID.Hand.kRight)),
                    robotDrive
            )
    );


  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      driverA_Button.whenPressed(new Spin(colorSubsystem, 3));


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    Constants.DriveConstants.kS,
                    Constants.DriveConstants.kV,
                    Constants.DriveConstants.kA
                    ),
            Constants.DriveConstants.kinematics,
            10
            );

    TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.maxVel,
            Constants.AutoConstants.maxAccel)
            .setKinematics(Constants.DriveConstants.kinematics)
            .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1,1),
                    new Translation2d(2,-1)
                    ),
            new Pose2d(3, 0, new Rotation2d(0)), config);

    RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory, robotDrive::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV, Constants.DriveConstants.kA),
            Constants.DriveConstants.kinematics,
            robotDrive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDrive,0,0),
            new PIDController(Constants.DriveConstants.kPDrive,0,0),
            robotDrive::setVoltage,
            robotDrive);

    // An ExampleCommand will run in autonomous
    return ramseteCommand.andThen(() -> robotDrive.setVoltage(0,0));
  }
}
