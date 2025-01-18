package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;

/**
 * Classe de constantes.
 */
public final class Constants {

    // Classe para definir dimensões e propriedades físicas do robô.
    public static final class Dimensoes {
        // Tempo de loop (sparkMax + normal = 130ms)
        public static final double LOOP_TIME = 0.13;
        // Massa do robô em quilogramas.
        public static final double ROBOT_MASS = 31;
        // Velocidade máxima do robô em metros por segundo.
        public static final double MAX_SPEED = 5;

        // Posições do centro de massa em metros.
        private static final double xMass = 13;
        private static final double yMass = 13;
        private static final double zMass = 0.08;

        // Centro de massa do chassi.
        public static final Matter CHASSIS = new Matter(new Translation3d(xMass, yMass, zMass), ROBOT_MASS);
    }

    // Classe que contém as configurações de PID para o modo autônomo.
    public static final class PID {
        // PID para frente e para trás.
        public static final PIDFConfig xAutoPID = new PIDFConfig(10.0, 0, 5);
        // PID para esquerda e direita.
        public static final PIDFConfig yAutoPID = new PIDFConfig(20, 0, 8.4);
        // PID de rotação.
        public static final PIDFConfig angleAutoPID = new PIDFConfig(8, 0, 5);
    }

    // Classe que define as configurações do controle (joystick).
    public static final class Controle {
        // Porta do controle (joystick).
        public static final int xboxControle = 0;

        // Deadband do controle, define a faixa de valores do joystick que serão ignorados para evitar movimentos acidentais.
        public static final double DEADBAND = 0.15;
    }

    // Classe que define as configurações de tração do robô.
    public static final class Tracao {
        // Define se a tração será orientada ao campo (true) ou ao robô (false).
        public static final boolean fieldRelative = true;
        // Define se o controle será em malha aberta (true) ou fechada (false).
        public static final boolean isOpenLoop = false;
        // Define se a correção de aceleração estará ativa.
        public static final boolean accelCorrection = false;
        // Constante para diminuir o input do joystick para rotação (0 < multiplicadorRotacional <= 1).
        public static final double multiplicadorRotacional = 0.8;
        // Constante para diminuir o input do joystick para translação no eixo Y.
        public static final double multiplicadorTranslacionalY = 0.7;
        // Constante para diminuir o input do joystick para translação no eixo X.
        public static final double multiplicadorTranslacionalX = 0.7;

        // Constante que define a sensibilidade da rotação.
        public static final double TURN_CONSTANT = 1.0;

        // Velocidade máxima do robô em metros por segundo.
        public static final double MAX_SPEED = 5;

        // Intervalo de tempo (em segundos) para atualização do controle.
        public static final double dt = 10.8;

        // Constante para rotação contínua.
        public static final double constantRotation = 4;
    }

    // Classe que guarda os nomes das trajetórias utilizadas no modo autônomo.
    public static final class Trajetoria {
        // Define se a aliança é azul (false) ou vermelha (true).
        public static final boolean ALIANCA = false;
        // Nome da primeira trajetória.
        public static final String NOME_TRAJETORIA = "AutoBack";
        // Nome da segunda trajetória.
        public static final String NOME_TRAJETORIA2 = "PID";
        public static final String NOME_TRAJETORIA3 = "AutoConesTeste";
    }
    public static final double DriveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4),
       6.75, 1);
  public static final double SteeringConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(21.4285714286, 1);
  public static final  PIDConstants TranslationPID = new PIDConstants(6, 0, 0);
  public static final PIDConstants angleAutoPID = new PIDConstants(5.5, 0, 0);
}
