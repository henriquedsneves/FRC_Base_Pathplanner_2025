// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.Teleop;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter; // Certifique-se de que esta importação está correta
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class RobotContainer {
  // Subsistemas
  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private Shooter shooter = new Shooter(); // Instância da classe Shooter

  // Controle de Xbox
  private XboxController controleXbox = new XboxController(Controle.xboxControle);

  public RobotContainer() {
    // Comando padrão para dirigir o swerve
    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(controleXbox.getLeftY(), Constants.Controle.DEADBAND),
      () -> MathUtil.applyDeadband(controleXbox.getLeftX(), Constants.Controle.DEADBAND),
      () -> controleXbox.getRightX()));

    // Registrar comandos nomeados
    NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
    NamedCommands.registerCommand("Shooter", new InstantCommand(() -> {
      System.out.println("Shoter Heading " + shooter.getHeading());
    }));

    // Configurar bindings
    configureBindings();
  }

  // Configuração de eventos (triggers)
  private void configureBindings() {
    // Adicione quaisquer triggers aqui, se necessário
  }

  // Retorna o comando autônomo
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(Trajetoria.NOME_TRAJETORIA, true);
  }

  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
