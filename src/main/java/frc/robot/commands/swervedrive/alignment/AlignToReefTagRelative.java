// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.alignment;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefAlignment;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReefTagRelative extends Command {
  private final PIDController xController, yController, rotController;
  private final boolean isRightScore;
  private final Timer dontSeeTagTimer, stopTimer;
  private final SwerveSubsystem drivebase;
  private final Vision vision;
  private double tagID = -1;

  /**
   * Constructs a new AlignToReefTagRelative command.
   *
   * @param isRightScore True for right scoring position, false for left.
   * @param drivebase The swerve drive subsystem.
   * @param vision The vision subsystem for PhotonVision.
   */
  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase, Vision vision) {
    xController = new PIDController(ReefAlignment.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
    yController = new PIDController(ReefAlignment.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horizontal movement
    rotController = new PIDController(ReefAlignment.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.vision = vision;
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

    rotController.setSetpoint(ReefAlignment.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(ReefAlignment.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(ReefAlignment.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(ReefAlignment.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? ReefAlignment.Y_SETPOINT_REEF_ALIGNMENT : -ReefAlignment.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(ReefAlignment.Y_TOLERANCE_REEF_ALIGNMENT);

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
        dontSeeTagTimer.reset();

        Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();
        double x = camToTarget.getX(); // Forward/backward
        double y = camToTarget.getY(); // Left/right
        double rot = camToTarget.getRotation().getZ(); // Yaw (radians)

        SmartDashboard.putNumber("x", x);

        double xSpeed = xController.calculate(x);
        SmartDashboard.putNumber("xspeed", xSpeed);
        double ySpeed = -yController.calculate(y);
        double rotValue = -rotController.calculate(rot);

        drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

        if (!rotController.atSetpoint() ||
            !yController.atSetpoint() ||
            !xController.atSetpoint()) {
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
    return dontSeeTagTimer.hasElapsed(ReefAlignment.DONT_SEE_TAG_WAIT_TIME) ||
    stopTimer.hasElapsed(ReefAlignment.POSE_VALIDATION_TIME);
  }
}
