package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableDouble {
    private String key;
    private double value;

    public TunableDouble(String key, double value) {
        this.key = key;
        this.value = value;

        SmartDashboard.putNumber(key, value);
    }

    public double get() {
        return value;
    }

    public boolean update() {
        if (value != SmartDashboard.getNumber(key, value)) {
            value = SmartDashboard.getNumber(key, value);
            
            return true;
        }
        else {
            return false;
        }
    }
}
