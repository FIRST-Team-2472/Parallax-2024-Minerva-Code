package frc.robot.subsystems.swerveExtras;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.RobotState;

public class RobotStateEstimator {
    static RobotStateEstimator INSTANCE;
    private RobotState robotState = RobotState.getInstance();

    private SwerveDrivePoseEstimator poseEstimator;

    /**
     * RobotStateEstimator uses SwerveDrivePoseEstimator to estimate the pose of the robot, field relative.
     */
    public static RobotStateEstimator getInstance(){
        if (INSTANCE == null) {
            INSTANCE = new RobotStateEstimator();
        }

        return INSTANCE;
    }

    private RobotStateEstimator() {}

    /**
     * Update the SwerveDrivePoseEstimator with values from RobotState
     */
    public void updateRobotPoseEstimator() {
        if (!robotState.hasValidSwerveState()) {
            return;
        }

        if(poseEstimator == null){
            poseEstimator = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kDriveKinematics, 
                robotState.getGyroRotation(), 
                robotState.getModulePositions(),
                robotState.getInitialPose()
                //,Constants.Drivetrain.ODOMETRY_STDDEV,
                //Constants.Vision.VISION_STDDEV
            );
        }
        
        /*
         *  Odometry updates
         */
        if(!robotState.getCollisionDetected()){
            poseEstimator.update(
                robotState.getGyroRotation(), 
                robotState.getModulePositions()
            );
        }
    }


    /**
     * Reset the position of SwerveDrivePoseEstimator and set the NavX Offset
     */
    public void resetOdometry(Pose2d pose){
        //robotState.resetInitialPose(pose);
        if(poseEstimator != null){
            poseEstimator.resetPosition(
                robotState.getGyroRotation(), 
                robotState.getModulePositions(),
                pose
            );
        }

    }
}
