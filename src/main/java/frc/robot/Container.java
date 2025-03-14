// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;

public class Container {
        Drivetrain drivetrain;
        Arm arm;
        Elevator elevator;
        Vision vision;

        CANdle lights;

        Mode mode;
        Elevator.Position coralLevel;
        Elevator.Position algaeLevel;

        public enum Mode {
                Coral,
                Algae
        }

        public Container() {
                drivetrain = new Drivetrain(
                        Constants.Drivetrain.drivetrainConfigs,

                        Constants.Drivetrain.frontLeftConfigs,
                        Constants.Drivetrain.frontRightConfigs,
                        Constants.Drivetrain.backLeftConfigs,
                        Constants.Drivetrain.backRightConfigs
                );

                NamedCommands.registerCommand("intake", runIntake());
                //NamedCommands.registerCommand("outtakeL4", return new CommandrunOuttake());


                arm = new Arm(Constants.Arm.pivotID, Constants.Arm.rollersID, Constants.Arm.distanceID);
                elevator = new Elevator(Constants.Elevator.leftID, Constants.Elevator.rightID, Constants.canivoreID);

                vision = new Vision(
                        Constants.Vision.frontID,

                        Constants.Vision.frontLeftID, Constants.Vision.frontLeftOffset,
                        Constants.Vision.frontRightID, Constants.Vision.frontRightOffset,
                        Constants.Vision.backLeftID, Constants.Vision.backLeftOffset,
                        Constants.Vision.backRightID, Constants.Vision.backRightOffset
                );

                lights = new CANdle(Constants.lightsID);

                mode = Mode.Coral;
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
        }

        public Drivetrain getDrivetrain() {
                return drivetrain;
        }

        public Arm getArm() {
                return arm;
        }

        public Elevator getElevator() {
                return elevator;
        }

        public Vision getVision() {
                return vision;
        }

        public Mode getMode() {
                return mode;
        }

        public Elevator.Position getCoralLevel() {
                return coralLevel;
        }

        public Elevator.Position getAlgaeLevel() {
                return algaeLevel;
        }

        public Pose2d getRobotPose() {
                Pose2d estimate = drivetrain.getState().Pose;
                
                return new Pose2d(
                        estimate.getX(),
                        estimate.getY(),
                        drivetrain.getRotation3d().toRotation2d()
                );
        }

        public void updateRobotPose(Pose2d estimate) {
                if (Utilities.isValidPose(estimate)) drivetrain.addVisionMeasurement(estimate, Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()));
        }

        public void updateRobotPose(Pose2d[] estimates) {
                for(Pose2d estimate : estimates) {
                       updateRobotPose(estimate);
                }
        }

        public void updateLEDs() {
                if (mode == Mode.Coral) lights.setLEDs(255, 0, 255);
                else lights.setLEDs(0, 255, 0);
        }

        public void modeCoral() {
                mode = Mode.Coral;
        }

        public void modeAlgae() {
                mode = Mode.Algae;
        }

        public void targetLow() {
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
        }

        public void targetMedium() {
                coralLevel = Elevator.Position.L3_Coral;
        }

        public void targetHigh() {
                coralLevel = Elevator.Position.L4_Coral;
                algaeLevel = Elevator.Position.High_Algae;
        }

        public Command driveJoysticks(double leftX, double leftY, double rightX, boolean slowed) {
                ChassisSpeeds speeds = new ChassisSpeeds(
                        -leftY * Constants.Drivetrain.maxSpeed,
                        -leftX * Constants.Drivetrain.maxSpeed,
                        -rightX * Constants.Drivetrain.maxAngularSpeed
                );
                
                Command command = drivetrain.driveSpeeds(speeds, slowed).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(drivetrain);

                return command;
        }

        public Command stow() {
                Command command = Commands.sequence(
                        Commands.either(
                                arm.setPosition(Arm.Position.Hold_Algae),
                                arm.setPosition(Arm.Position.Stow),
                                
                                () -> arm.hasAlgae()
                        ),
                        elevator.setPosition(Elevator.Position.Stow)
                ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command runIntake() {
                Command command = Commands.sequence(
                        arm.setPosition(Arm.Position.Stow),
                        Commands.either(
                                Commands.sequence(
                                        elevator.setPosition(Elevator.Position.Stow),
                                        Commands.parallel(
                                                arm.setPosition(Arm.Position.Intake_Coral),
                                                arm.intakeCoral()
                                        ),
                                        arm.setPosition(Arm.Position.Stow)
                                ),
                                Commands.sequence(
                                        Commands.parallel(
                                                arm.setPosition(Arm.Position.Hold_Algae),
                                                elevator.setPosition(algaeLevel),
                                                arm.intakeAlgae()
                                        )
                                ),

                                () -> mode == Mode.Coral
                        )
                ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command runOuttake() {
                Command command = Commands.either(
                        Commands.either(
                                arm.outtakeCoral(),
                                Commands.sequence(
                                        arm.setPosition(Arm.Position.Stow),
                                        elevator.setPosition(coralLevel),
                                        arm.setPosition(coralLevel == Elevator.Position.L4_Coral ? Arm.Position.L4_Coral : Arm.Position.Stow)
                                ),
                                
                                () -> Utilities.inTolerance(coralLevel.value - elevator.getPosition(), 0.4)
                        ),
                        Commands.either(
                                Commands.parallel(
                                        Commands.sequence(
                                                elevator.setPosition(Elevator.Position.End_Barge),
                                                Commands.waitSeconds(0.2),
                                                arm.setPosition(Arm.Position.Stow)
                                        ),
                                        Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                arm.outtakeAlgae(-1)
                                        )
                                ),
                                Commands.either(
                                        arm.outtakeAlgae(-0.4),
                                        Commands.sequence(
                                                arm.setPosition(Arm.Position.Hold_Algae),
                                                Commands.sequence(
                                                        elevator.setPosition(Elevator.Position.Start_Barge),
                                                        arm.setPosition(Arm.Position.Start_Barge)
                                                )
                                        ),

                                        () -> Utilities.inTolerance(Elevator.Position.Stow.value - elevator.getPosition(), 0.4)
                                ),

                                () -> Utilities.inTolerance(Elevator.Position.Start_Barge.value - elevator.getPosition(), 0.4)
                        ),

                        () -> mode == Mode.Coral
                ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command climb() {
                Command command = Commands.either(
                        Commands.sequence(
                                elevator.setPosition(Elevator.Position.Stow)
                        ),
                        Commands.sequence(
                                arm.setPosition(Arm.Position.Stow),
                                elevator.setPosition(Elevator.Position.Start_Climb)
                        ),

                        () -> Utilities.inTolerance(Elevator.Position.Start_Climb.value - elevator.getPosition(), 0.4)
                ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                command.addRequirements(arm,  elevator);
                
                return command;
        }
        public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto("New Auto");
  }
}
