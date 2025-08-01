// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClawElevatorSequentialCommands;
import frc.robot.commands.ClawMoveCommand;
import frc.robot.commands.ElevatorClawSequentialCommands;
import frc.robot.commands.ElevatorMoveComand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
//Caleb Lopez did 95% of the code
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Claw claw = new Claw(); 
  private final AlgaeIntake algaeIntakeSubsystem = new AlgaeIntake(); 
  private final Elevator elevator = new Elevator();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private static CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    public static boolean getLeftBumper(){
        return operatorController.leftBumper().getAsBoolean();
    }
    public static Boolean getRightBumper(){
        return operatorController.rightBumper().getAsBoolean();
    }
    public static double getRightTrigger(){
        return operatorController.getRightTriggerAxis();
    }
    public static double getLeftTrigger(){
        return operatorController.getLeftTriggerAxis();
    }
    public static double getRightYValue(){
        return operatorController.getRightY();
    }
    public static boolean getBButton(){
        return operatorController.b().getAsBoolean();
    }
    public static boolean getAButton(){
        return operatorController.b().getAsBoolean();
    }
    public static boolean getXButton(){
        return operatorController.x().getAsBoolean();
    }
    public static boolean getYButton(){
        return operatorController.y().getAsBoolean();
    }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.p
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    // new Trigger(() -> Math.abs(operatorController.getRightY()) > OIConstants.kOperatorDeadband) // Deadband of 0.1 to prevent drift
    //     .whileTrue(new RunCommand(() -> {
    //         double speed = -operatorController.getRightY(); // Invert if needed
    //         claw.setClawSpeed(speed); // Create this method in Claw subsystem
    //     }, claw));
    // new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
    //     .whileTrue(new RunCommand(() -> algaeIntakeSubsystem.Intake(), algaeIntakeSubsystem));

    // Right bumper (kBumperRight) for Eject
    operatorController.b().onTrue(new ClawElevatorSequentialCommands(claw, elevator, 1, -31)); //reset position
    operatorController.a().onTrue(new ElevatorClawSequentialCommands(elevator, claw, 11, -17)); //L2
    operatorController.x().onTrue(new ElevatorClawSequentialCommands(elevator, claw, 39, -17)); //L3
    operatorController.y().onTrue(new ElevatorClawSequentialCommands(elevator, claw, 92.5, -17)); //L4

    // operatorController.b().onTrue(new ElevatorMoveComand(elevator, claw, -31, 1)); //reset position
    // operatorController.a().onTrue(new ElevatorMoveComand(elevator, claw, 11, -14)); //L2
    // operatorController.x().onTrue(new ElevatorMoveComand(elevator, claw, 39, -14)); //L3
    // operatorController.y().onTrue(new ElevatorMoveComand(elevator, claw, 92.5, -14)); //L4
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {



    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return new InstantCommand(() -> elevator.SetPos(1.0), elevator)
        .andThen(new InstantCommand(() -> claw.SetPos(1.0), claw));
  }
}
//Teagan(braxton)(helen) did the other 5%
// realllllll