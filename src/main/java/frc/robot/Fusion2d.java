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
    private Pigeon2 pigeon;
    private Pose2d pose = new Pose2d();
    private double[] linearVelocity = { 0, 0 };
    private double[] linearPosition = { 0, 0 };
    private float loopTime = 0.02f;
    private double yawOffset = 0.0d;

    public Fusion2d(Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }

    public void resetPosition(Rotation2d rotation, Pose2d pose) {
        this.yawOffset = rotation.getDegrees() - this.pose.getRotation().getDegrees();
        this.pose = new Pose2d(pose.getTranslation(), rotation);
    }

    public void resetPosition(Pose2d pose) {
        this.yawOffset = pose.getRotation().getDegrees() - this.pose.getRotation().getDegrees();
        this.pose = pose;
    }

    // s = s + u * dt;
    // u = u + a * dt;

    public void update() {
        this.linearVelocity[0] = this.linearVelocity[0]
                + (this.pigeon.getAccelerationX().getValueAsDouble() - this.pigeon.getGravityVectorX().getValueAsDouble()) * 9.80665 * this.loopTime;
        this.linearPosition[0] = this.linearPosition[0] + this.linearVelocity[0] * this.loopTime;

        this.linearVelocity[1] = this.linearVelocity[1]
                + (this.pigeon.getAccelerationY().getValueAsDouble() - this.pigeon.getGravityVectorY().getValueAsDouble()) * 9.80665 * this.loopTime;
        this.linearPosition[1] = this.linearPosition[1] + this.linearVelocity[1] * this.loopTime;

        this.pose = new Pose2d(this.linearPosition[0], this.linearPosition[1],
                Rotation2d.fromDegrees(this.pigeon.getYaw().getValueAsDouble() + this.yawOffset));
    }

    public Pose2d getPoseMeters() {
        return this.pose;
    }

}
