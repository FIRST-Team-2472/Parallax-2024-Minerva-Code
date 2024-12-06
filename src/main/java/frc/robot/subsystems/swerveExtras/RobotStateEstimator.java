package frc.robot.subsystems.swerveExtras;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.RobotState;

public class RobotStateEstimator {
    public static Pose2d getEstimatedPose2d(){
        double x = (RobotState.cameraPose.getX()*Constants.TargetPosConstants.kCameraWeight + 
        RobotState.swerveOdometryPose.getX()*Constants.TargetPosConstants.kSwerveWeight)/
        (Constants.TargetPosConstants.kSwerveWeight+ Constants.TargetPosConstants.kCameraWeight);
        return new Pose2d();
    }
}
