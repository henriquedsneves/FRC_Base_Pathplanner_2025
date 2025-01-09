package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

/** Configuração para o modo autônomo. */
public class ConfigAuto {

    SwerveSubsystem swerve; // Declaração do subsistema Swerve

    // Construtor que inicializa o subsistema Swerve
    public ConfigAuto(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }
    // Método para configurar o Path Planner
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            swerve::getPose, // Fornecedor da pose do robô
            swerve::resetOdometry, // Método para resetar a odometria
            swerve::getRobotVelocity, // Fornecedor das velocidades do chassis, deve ser relativo ao robô
            swerve::setChassisSpeeds, // Método para definir as velocidades do chassis relativo ao robô
            new HolonomicPathFollowerConfig( // Configuração do HolonomicPathFollower
                new PIDConstants(0.8, 0.0, 0.02), // Constantes PID para translação
                new PIDConstants(
                    swerve.getSwerveController().config.headingPIDF.p,
                    swerve.getSwerveController().config.headingPIDF.i,
                    swerve.getSwerveController().config.headingPIDF.d
                ), // Constantes PID para rotação
                4.0, // Velocidade máxima do módulo em m/s
                swerve.getSwerveDriveConfiguration().getDriveBaseRadiusMeters(), // Raio da base de direção
                new ReplanningConfig() // Configuração padrão de replanejamento do caminho
            ),
            () -> {
                // Determina se o caminho será espelhado para a aliança vermelha
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : true;
            },
            swerve // Referência a este subsistema para definir dependências
        );
    }
   

}
