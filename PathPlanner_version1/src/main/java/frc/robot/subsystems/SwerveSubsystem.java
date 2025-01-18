package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Tracao;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

/** 
 * Classe de subsistema onde fazemos a ponte do nosso código para YAGSL
 */
public class SwerveSubsystem extends SubsystemBase {

    double velocidadeAlinhamento = 0.12;
  
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.01, 0, 0) ;
  PIDController rotationPID = new PIDController(0.01, 0.00, 0.00);
  double velocidade = 0.2;
  double output;
 

    // Objeto global da SwerveDrive (Classe YAGSL)
    public SwerveDrive swerveDrive;
    Constants constants;
    // Método construtor da classe
    public SwerveSubsystem(File directory) {
        // Seta a telemetria como nível mais alto
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Acessa os arquivos do diretório .JSON
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Dimensoes.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(true); // Ativa a correção de direção
        setupPathPlanner(); // Configura o Path Planner
       // swerveDrive.setChassisDiscretization(true, Constants.Tracao.dt);
       //swerveDrive = new SwerveParser(directory).createSwerveDrive(Tracao.MAX_SPEED);
        swerveDrive.setAngularVelocityCompensation(true, true, 1.2);
        SwerveDriveTest.centerModules(swerveDrive);
    }
    
    @Override
    public void periodic() {
       // System.out.println("Angulo do robo:" + getHeading());
       //System.out.println(getPose());
        // Dentro da função periódica atualizamos nossa odometria
        swerveDrive.updateOdometry();
        
        
    }
   

    public void setupPathPlanner() {
         // Calcular o valor do feedforward para a velocidade máxima
    double feedforwardValue = feedforward.calculate(Constants.Dimensoes.MAX_SPEED);
        AutoBuilder.configureHolonomic(
            this::getPose, // Fornecedor da pose do robô
            this::resetOdometry, // Método para resetar a odometria
            this::getRobotVelocity, // Fornecedor das velocidades do chassis, deve ser relativo ao robô
            this::setChassisSpeeds, // Método para definir as velocidades do chassis relativo ao robô
            new HolonomicPathFollowerConfig( // Configuração do HolonomicPathFollower
                new PIDConstants(feedforwardValue, 0.0, 0.0), // Constantes PID para translação
                new PIDConstants(
                    rotationPID.getP(),
                    rotationPID.getI(),
                    rotationPID.getD()
                ), // Constantes PID para rotação
                4.0, // Velocidade máxima do módulo em m/s
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Raio da base de direção
                new ReplanningConfig() // Configuração padrão de replanejamento do caminho
            ),
            () -> {
                // Determina se o caminho será espelhado para a aliança vermelha
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this // Referência a este subsistema para definir dependências
        );
    }

    // Movimenta o robô com o joystick esquerdo, e mira o robô no ângulo no qual o joystick está apontando
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                                DoubleSupplier headingY) {
        return run(() -> {
            double xInput = Math.pow(translationX.getAsDouble(), 3);
            double yInput = Math.pow(translationY.getAsDouble(), 3);
            // Faz o robô se mover
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                            headingX.getAsDouble(),
                                                                            headingY.getAsDouble(),
                                                                            swerveDrive.getYaw().getRadians(),
                                                                            swerveDrive.getMaximumVelocity()));
        });
    }

    // Movimenta o robô com o joystick esquerdo, e gira o robô na intensidade na qual o joystick direito está para o lado
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            double xInput = Math.pow(translationX.getAsDouble(), 3);
            double yInput = Math.pow(translationY.getAsDouble(), 3);
            // Faz o robô se mover
            swerveDrive.drive(new Translation2d(xInput * swerveDrive.getMaximumVelocity(),
                                                yInput * swerveDrive.getMaximumVelocity()),
                              angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                              true,
                              false);
        });
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
        
    }

    // Função drive que chamamos em nossa classe de comando Teleoperado
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    // Função para obter a velocidade desejada a partir dos inputs do gamepad
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, 
                                                            getHeading().getRadians());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) {
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 0, 0, 0, Tracao.MAX_SPEED);
    }

    // Função que retorna a posição do robô (translação e ângulo), (Usado no autônomo)
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }
  
    // Retorna a velocidade relativa ao campo
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    // Retorna a configuração do swerve
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    // Retorna o objeto de controle, o qual é usado para acessar as velocidades máximas por exemplo
    public SwerveController getSwerveController() {
        return swerveDrive.getSwerveController();
    }

    // Ângulo atual do robô
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    // Reseta a odometria para uma posição indicada (Usado no autônomo)
    public void resetOdometry(Pose2d posicao) {
        swerveDrive.resetOdometry(posicao);
    }

    // Seta a velocidade do chassi (Usado no autônomo)
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }
     public void stop() {
        System.out.println("Swerve Subsystem: Robô parado.");
        swerveDrive.drive( new Translation2d(0.0, 0.0), 0.0,true, false );
        // Lógica para parar os motores do swerve
    }

    

    
    public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
    //     Cria um comando para seguir um caminho usando o AutoBuilder. Isso também acionará marcadores de eventos.
       return new PathPlannerAuto(pathName);
     }
}


