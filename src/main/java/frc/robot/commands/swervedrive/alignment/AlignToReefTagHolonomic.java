// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.alignment;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ReefAlignment;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToReefTagHolonomic extends Command {
  private HolonomicDriveController controller;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private Vision vision;
  private double tagID = -1;

  /**
  *
  * @param isRightScore True for right scoring position, false for left.
  * @param drivebase The swerve drive subsystem.
  * @param vision The vision subsystem for PhotonVision.
  */
  public AlignToReefTagHolonomic(boolean isRightScore, SwerveSubsystem drivebase, Vision vision) {
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.vision = vision;

    // PID controllers for x, y, and theta
    PIDController xController = new PIDController(ReefAlignment.X_REEF_ALIGNMENT_P, 0.0, 0);
    PIDController yController = new PIDController(ReefAlignment.Y_REEF_ALIGNMENT_P, 0.0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      ReefAlignment.ROT_REEF_ALIGNMENT_P, 0.0, 0,
        new TrapezoidProfile.Constraints(3.5, 2.5)); // Max angular velocity and acceleration

    controller = new HolonomicDriveController(xController, yController, thetaController);
    dontSeeTagTimer = new Timer();
    stopTimer = new Timer();

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopTimer.reset();
    stopTimer.start();
    dontSeeTagTimer.reset();
    dontSeeTagTimer.start();

    PhotonPipelineResult result = Cameras.CENTER_CAM.getBestResult().orElse(null);
    if (result != null && result.hasTargets()) {
      tagID = result.getBestTarget().getFiducialId();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        PhotonPipelineResult result = Cameras.CENTER_CAM.getBestResult().orElse(null);
    if (result != null && result.hasTargets() && result.getBestTarget().getFiducialId() == tagID) {
      this.dontSeeTagTimer.reset();

      Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();
      Transform3d robotToCam = Cameras.CENTER_CAM.robotToCamTransform; // From Vision enum
      Transform3d tagToCam = camToTarget.inverse();
      Transform3d cameraToRobot = robotToCam.inverse();
      Transform3d tagToRobot = tagToCam.plus(cameraToRobot);

      double x = tagToRobot.getX(); // Forward/Backward
      double y = tagToRobot.getY(); // Left/Right
      double rot = tagToRobot.getRotation().getZ(); // Yaw

      SmartDashboard.putNumber("x", x);

      Pose2d currentPose = new Pose2d(x, y, new Rotation2d(rot));
      Pose2d goalPose = new Pose2d(
        ReefAlignment.X_SETPOINT_REEF_ALIGNMENT,
          isRightScore ? ReefAlignment.Y_SETPOINT_REEF_ALIGNMENT : -ReefAlignment.Y_SETPOINT_REEF_ALIGNMENT,
          new Rotation2d(ReefAlignment.ROT_SETPOINT_REEF_ALIGNMENT));

      ChassisSpeeds speeds = controller.calculate(currentPose, goalPose, 0.0, goalPose.getRotation());

      drivebase.drive(
          new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
          speeds.omegaRadiansPerSecond,
          false);

      if (!controller.atReference()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(ReefAlignment.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(ReefAlignment.POSE_VALIDATION_TIME);
  }
}
