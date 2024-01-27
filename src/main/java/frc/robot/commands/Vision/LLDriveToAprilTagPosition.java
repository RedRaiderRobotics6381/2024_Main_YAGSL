// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Vision;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// //import edu.wpi.first.math.geometry.Translation2d;
// //import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.LimelightHelpers;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import edu.wpi.first.wpilibj.XboxController;

// public class LLDriveToAprilTagPosition extends CommandBase {

//   private final SwerveSubsystem swerveSubsystem;
//   private final PIDController   yController;
//   private final PIDController   xController;
//   private final PIDController   zController;
//   private double visionObject;
  
//   public LLDriveToAprilTagPosition(SwerveSubsystem swerveSubsystem, double visonObject, int aprilTagID) {
//     this.swerveSubsystem = swerveSubsystem;
//     yController = new PIDController(0.1, 0.0, 0.0);
//     xController = new PIDController(0.1, 0.0, 0.0);
//     zController = new PIDController(0.1, 0.0, 0.0);
//     yController.setTolerance(.5);
//     xController.setTolerance(.5);
//     zController.setTolerance(.5);
//     yController.setSetpoint(0.0);
//     xController.setSetpoint(0.0);
//     zController.setSetpoint(0.0);
//     // each subsystem used by the command must be passed into the
//     // addRequirements() method (which takes a vararg of Subsystem)
//     addRequirements(this.swerveSubsystem);
//     this.visionObject = visionObject;
//   }

//   @Override
//   public void initialize() {
//     NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(visionObject);
//   }

//   @Override
//   public void execute() {
//     NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    

//     NetworkTableEntry targetValid = table.getEntry("tv"); 
//     //SmartDashboard.putBoolean("At Tolerance", yController.atSetpoint());
//     double tv = targetValid.getDouble(0.0);

//     // SmartDashboard.putNumber("Pipeline",tv);
//     // SmartDashboard.putBoolean("TV", tv);
//     if (tv == 1){
//       RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0.25);
//       // NetworkTableEntry targetX = table.getEntry("tx");
//       // NetworkTableEntry targetY = table.getEntry("ty");
//       // NetworkTableEntry targetZ = table.getEntry("tx");
//       // double tx = targetX.getDouble(0.0);
//       // double ty = targetY.getDouble(0.0);
//       // double tz = targetZ.getDouble(0.0);
//       double tx = LimelightHelpers.getTX("");
//       double ty = LimelightHelpers.getTY("");
//       double tz = LimelightHelpers.getTX("");
//       double target[] = LimelightHelpers.getTargetPose_CameraSpace("");

//       //double throttle = RobotContainer.driverXbox.getLeftTriggerAxis();

//       // This is the value in meters per second that is used to drive the robot

//       // double translationValy = MathUtil.clamp(yController.calculate(tx, 0.0), -.5 , .5); //* throttle, 2.5 * throttle);
//       // double translationValx = MathUtil.clamp(xController.calculate(ty, 0.0), -.5 , .5); //* throttle, 2.5 * throttle);
//       // double translationValz = MathUtil.clamp(zController.calculate(tz, 0.0), -.5 , .5); //* throttle, 2.5 * throttle);

//       double translationValy = yController.calculate(tx, 0.0); //* throttle, 2.5 * throttle);
//       double translationValx = xController.calculate(ty, 0.0); //* throttle, 2.5 * throttle);
//       double translationValz = zController.calculate(tz, 0.0); //* throttle, 2.5 * throttle);
//       SmartDashboard.putNumber("Y Translation Value", translationValy);
//       SmartDashboard.putNumber("X Translation Value", translationValx);
//       SmartDashboard.putNumber("Z Translation Value", translationValz);
      
//       swerveSubsystem.drive(new Translation2d(-target[1], target[0]), 0.0, false);
//       //swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(-translationValx, 0.0, translationValz));
//       //swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, translationValy,-translationValx));

//     }
//     else{
//       //swerveSubsystem.drive(new Translation2d(0, 0), 0.0, false, false);
//       end(true);
//       }
    
//       // double translationVal = MathUtil.clamp(controller.calculate(swerveSubsystem.getPitch().getDegrees(), 0.0), -0.5,
//     //                                        0.5);
//     // swerveSubsystem.drive(new Translation2d(translationVal, 0.0), 0.0, true, false);

//   }

//   @Override
//   public void end(boolean interrupted) {
//     //swerveSubsystem.lock();
//     RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0);
//   }

//   @Override
//   public boolean isFinished() {
//     return yController.atSetpoint() && xController.atSetpoint();
//   }
// }
