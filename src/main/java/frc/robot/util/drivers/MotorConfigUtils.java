package frc.robot.util.drivers;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class MotorConfigUtils {
    public static final int POSITION_SLOT_IDX = 0;
    public static final int VELOCITY_SLOT_IDX = 1;

    public static void setDefaultTalonFXConfig(TalonFX talonFX) {
        TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
        defaultConfig.voltageCompSaturation = 12;
        defaultConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(false, 0, 0, 0);
        defaultConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 0, 0, 0);
        defaultConfig.openloopRamp = 0;

        talonFX.configFactoryDefault();
        talonFX.configAllSettings(defaultConfig);
        talonFX.setNeutralMode(NeutralMode.Coast);
        talonFX.enableVoltageCompensation(true);
    }


}