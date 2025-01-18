package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Dimensoes;
import frc.robot.Constants.Tracao;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/** 
 * Classe que calcula a partir da entrada do gamepad a saída do swerve.
 */
public class Teleop extends Command {
    DoubleSupplier y; // Fornecedor de translação no eixo Y
    DoubleSupplier x; // Fornecedor de translação no eixo X
    DoubleSupplier turn; // Fornecedor de rotação

    SwerveSubsystem swerve; // Referência ao subsistema Swerve
    SwerveController controller; // Controlador Swerve

    Translation2d translation; // Translação do swerve
    double angle; // Ângulo
    double omega; // Velocidade angular

    // Construtor que inicializa as variáveis
    public Teleop(SwerveSubsystem swerve, DoubleSupplier y, DoubleSupplier x, DoubleSupplier turn) {
        this.y = y;
        this.x = x;
        this.turn = turn;
        this.swerve = swerve;
        controller = swerve.getSwerveController(); // Obtém o controlador Swerve

        // Adiciona o subsistema como dependência
        addRequirements(swerve);
    }

    // Chamado quando o comando é inicialmente agendado
    @Override
    public void initialize() {
    }

    // Calcula os valores de saída a partir dos inputs
    @Override
    public void execute() {
        double xVelocity = y.getAsDouble() * Tracao.multiplicadorTranslacionalY;
        double yVelocity = x.getAsDouble() * Tracao.multiplicadorTranslacionalX;
        double angVelocity = turn.getAsDouble() * Tracao.multiplicadorRotacional;

        translation = new Translation2d(xVelocity * Tracao.MAX_SPEED, yVelocity * Tracao.MAX_SPEED);
        omega = controller.config.maxAngularVelocity * angVelocity;

        // Limita a aceleração do robô
        
    }
  }