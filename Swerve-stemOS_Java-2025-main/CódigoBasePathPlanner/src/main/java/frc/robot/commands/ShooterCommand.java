package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    public ShooterCommand(SwerveSubsystem subsystem) {
        this.swerveSubsystem = subsystem;

        // Adiciona o subsistema como requisito
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // Inicializa o comando
        System.out.println("ShooterCommand iniciado");
    }

    @Override
    public void execute() {
        // Lógica principal do comando
        System.out.println("Encoder Value: " + swerveSubsystem.getSwerveController() + " ------- "+ swerveSubsystem.getHeading());
    }

    @Override
    public void end(boolean interrupted) {
        // Finaliza o comando
        System.out.println("ShooterCommand finalizado");
    }

    @Override
    public boolean isFinished() {
        // Retorna falso para que o comando continue executando
        return false;
    }
}
