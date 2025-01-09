package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Dimensoes;
import frc.robot.Constants.Tracao;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * Controle avançado de direção Swerve com 4 botões para diferentes direções.
 */
public class AbsoluteDriveAdv extends Command {

    private final SwerveSubsystem swerve; // Referência ao subsistema Swerve
    private final DoubleSupplier vX, vY; // Fornecedores de entrada de translação
    private final DoubleSupplier headingAdjust; // Fornecedor de ajuste de direção
    private boolean initRotation = false; // Flag para inicialização de rotação
    private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight; // Fornecedores booleanos para controlar a direção

    // Construtor para inicializar todas as variáveis
    public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                            BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft, BooleanSupplier lookRight) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingAdjust = headingAdjust;
        this.lookAway = lookAway;
        this.lookTowards = lookTowards;
        this.lookLeft = lookLeft;
        this.lookRight = lookRight;

        addRequirements(swerve); // Adiciona o subsistema como dependência
    }

    @Override
    public void initialize() {
        initRotation = true; // Inicializa a rotação
    }

    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;
        Rotation2d newHeading = Rotation2d.fromRadians(0);

        // Controla a direção do robô baseado nos botões
        if (lookAway.getAsBoolean()) {
            headingX = 1;
        }
        if (lookRight.getAsBoolean()) {
            headingY = 1;
        }
        if (lookLeft.getAsBoolean()) {
            headingY = -1;
        }
        if (lookTowards.getAsBoolean()) {
            headingX = -1;
        }

        // Ajusta a direção baseada no headingAdjust se nenhum botão estiver pressionado
        if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
            newHeading = Rotation2d.fromRadians(Tracao.TURN_CONSTANT * -headingAdjust.getAsDouble())
                    .plus(swerve.getHeading());
            headingX = newHeading.getSin();
            headingY = newHeading.getCos();
        }

        // Calcula as velocidades desejadas para o chassis
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                headingX, headingY);

        // Previne movimento após o modo autônomo
        if (initRotation) {
            if (headingX == 0 && headingY == 0) {
                Rotation2d firstLoopHeading = swerve.getHeading();
                desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
            }
            initRotation = false;
        }

        // Limita a velocidade para prevenir tombos
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

        // Limita a aceleração do robô
        if (Tracao.accelCorrection) {
            translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                    Dimensoes.LOOP_TIME, Dimensoes.ROBOT_MASS, List.of(Dimensoes.CHASSIS), swerve.getSwerveDriveConfiguration());
            SmartDashboard.putNumber("LimitedTranslation", translation.getX());
            SmartDashboard.putString("Translation", translation.toString());
        }

        // Faz o robô se mover
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, Tracao.fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
