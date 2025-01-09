package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tx, ty, ta, tv, tid;

    // Construtor que inicializa a tabela da Limelight
    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelightTable.getEntry("tx"); // Posição X da tag
        ty = limelightTable.getEntry("ty"); // Posição Y da tag
        ta = limelightTable.getEntry("ta"); // Área da tag
        tv = limelightTable.getEntry("tv"); // Se encontrou ou não uma tag
        tid = limelightTable.getEntry("tid"); // ID da tag detectada
    }

    // Método para obter o valor de tx (posição horizontal da tag)
    public double getTx() {
        return tx.getDouble(0.0);  // Retorna o valor de tx (0.0 se não tiver dado)
    }

    // Método para obter o valor de ty (posição vertical da tag)
    public double getTy() {
        return ty.getDouble(0.0);  // Retorna o valor de ty (0.0 se não tiver dado)
    }

    // Método para obter o valor de ta (área da tag)
    public double getTa() {
        return ta.getDouble(0.0);  // Retorna o valor de ta (0.0 se não tiver dado)
    }

    // Método para verificar se a Limelight encontrou uma tag
    public boolean hasTarget() {
        return tv.getDouble(0.0) == 1.0; // Se tv for 1.0, significa que há uma tag detectada
    }

    // Método para obter o ID da tag detectada
    public int getTagId() {
        return (int) tid.getDouble(-1.0); // Retorna o ID da tag ou -1 se não houver tag
    }
    

    @Override
    public void periodic() {
      
        // Exibe as variáveis no SmartDashboard para monitoramento
        SmartDashboard.putNumber("LimelightX", getTx());
        SmartDashboard.putNumber("LimelightY", getTy());
        SmartDashboard.putNumber("LimelightArea", getTa());
        SmartDashboard.putBoolean("HasTarget", hasTarget());
        SmartDashboard.putNumber("TagId", getTagId());
      
    }
    
}
