package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotState{
        public static Pose2d cameraPose = null;
        public static Pose2d swerveOdometryPose = null;
        public static Rotation2d gryoRotation = null;
        public static boolean noteSensorState = false;
        RobotState(){

        }
    }
