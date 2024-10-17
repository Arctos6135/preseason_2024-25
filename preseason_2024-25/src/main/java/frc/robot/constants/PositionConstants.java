package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PositionConstants {

    public static final Pose2d RED_AMP = new Pose2d(15.85, 6.7, Rotation2d.fromDegrees(180-60));
    public static final Pose2d RED_STAGE = new Pose2d(15.17, 5.57, Rotation2d.fromDegrees(180));
    public static final Pose2d RED_SOURCE = new Pose2d(15.17, 4.40, Rotation2d.fromDegrees(-120));

    public static final Pose2d BLUE_AMP = new Pose2d(0.68, 6.7, Rotation2d.fromDegrees(60));
    public static final Pose2d BLUE_STAGE = new Pose2d(1.35, 5.55, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_SOURCE = new Pose2d(0.68, 4.40, Rotation2d.fromDegrees(-60));
}
