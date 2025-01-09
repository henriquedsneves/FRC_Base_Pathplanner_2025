package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.Limelight;
import frc.robot.commands.Teleop;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
    // Inicializa os subsistemas
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    // Instância do comando Limelight
    private final Limelight limelightCommand = new Limelight(limelightSubsystem, swerve);

    private XboxController controleXbox = new XboxController(Controle.xboxControle);

    public RobotContainer() {
        swerve.setDefaultCommand(swerve.driveCommand(
            () -> -MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
            () -> -MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND),
            () -> controleXbox.getRightX()
        ));

        NamedCommands.registerCommand("LimeLight", new ParallelCommandGroup(
          new RunCommand(() -> limelightCommand.execute(), limelightSubsystem),
          new WaitCommand(2)
      ));}

    private void configureBindings() {
        // Adicione bindings de controles aqui
    }

    public Command getAutonomousCommand() {
        return swerve.getAutonomousCommand(Trajetoria.NOME_TRAJETORIA, true);
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }
}
