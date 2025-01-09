package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import swervelib.parser.PIDFConfig;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FullAuto1 extends AutoBuilder {

    private final Consumer<ChassisSpeeds> outputChassisSpeeds;
    private final Subsystem[] driveRequirements;
    private final PIDFConfig anglePIDController;
    private final PIDFConfig yPIDController;
    private final PIDFConfig xPIDController;
    private final PIDController anglePIDControllerWPI;

    public FullAuto1(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        PIDFConfig xPIDController,
        PIDFConfig yPIDController,
        PIDFConfig anglePIDController,
        Consumer<ChassisSpeeds> outputChassisSpeeds,
        Map<String, Command> eventMap,
        boolean useAllianceColor,
        Subsystem... swerveSubsystem) {

        super();

        this.xPIDController = xPIDController;
        this.yPIDController = yPIDController;
        this.anglePIDController = anglePIDController;
        this.outputChassisSpeeds = outputChassisSpeeds;
        this.driveRequirements = swerveSubsystem;

        this.anglePIDControllerWPI = new PIDController(
            anglePIDController.p,
            anglePIDController.i,
            anglePIDController.d
        );
        this.anglePIDControllerWPI.enableContinuousInput(-180, 180);
    }
   
    

}
