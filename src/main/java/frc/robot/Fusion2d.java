package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.Timer;

public class Fusion2d {
    private Pose2d pose;
    private Pigeon2 pigeon;
    private double[] linearVelocity = { 0, 0 };
    private float loopTime = 0.02f;

    public Fusion2d(Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }

    public void resetPosition(Rotation2d rotation, Pose2d pose) {
        this.pose = new Pose2d(pose.getTranslation(), rotation);
    }

    public void resetPosition(Pose2d pose) {
        this.pose = pose;
    }

    public void update() {
        this.linearVelocity[0] += this.pigeon.getAccelerationX().getValueAsDouble() * this.loopTime;
        this.linearVelocity[1] += this.pigeon.getAccelerationY().getValueAsDouble() * this.loopTime;
        double rotationalVelocity = this.pigeon.getYaw().getValueAsDouble() * this.loopTime;
        pose.transformBy(new Transform2d(
                new Translation2d(this.linearVelocity[0] * this.loopTime, this.linearVelocity[1] * this.loopTime),
                new Rotation2d(rotationalVelocity * this.loopTime)));
    }

    public Pose2d getPoseMeters() {
        return this.pose;
    }

}
