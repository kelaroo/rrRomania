package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class PowersConfig {

    public static final double COEFF_SPEED_HIGH = 0.8;
    public static final double COEFF_SPEED_LOW = 0.5;
    public static final double COEFF_ROTATE = 0.7;

    public static final double INTAKE_SUCK = 0.75;
    public static final double INTAKE3_SUCK = 0.75;

    public static final PIDFCoefficients lansatCoeff = new PIDFCoefficients(25, 0.0, 10, 12.8);
    @Deprecated static final double LANSAT_POWER = 0.675; //0.62;
    @Deprecated static final double LANSAT_POWER_PS = 0.575;
    public static final double LANSAT_POWER_AUTO = 0.65;
    public static final double LANSAT_SPEED = 1450;
    public static final double LANSAT_SPEED_PS = 1270;

    public static final double LANSAT_AUTO_PS = 1415; //1425
    public static final double LANSAT_AUTO_A1 = 1410;
    public static final double LANSAT_AUTO_B1 = 1420;
    public static final double LANSAT_AUTO_B2 = 1390;
    public static final double LANSAT_AUTO_C1 = 1480;
    public static final double LANSAT_AUTO_C2 = 1430;

    public static final double INTAKE2_RIGHT = 1;
    public static final double INTAKE2_LEFT = 0;
    public static final double INTAKE2_STATIONARY = 0.5;

    public static final double INTAKE3_RIGHT = 1;
    public static final double INTAKE3_LEFT = 0;
    public static final double INTAKE3_STATIONARY = 0.5;

    public static final double CUVA_SUS = 0.356;
    public static final double CUVA_JOS = 0.625;

    public static final double IMPINS_FWD = 0.00; // 0
    public static final double IMPINS_SECOND = 0.37; // 0.3
    public static final double IMPINS_BWD = 0.27; // 0.25

    public static final double BARAD_EXT = 0.08;
    public static final double BARAD_INT = 0.4;
    public static final double BARAS_EXT = 0.75;
    public static final double BARAS_INT = 0.37;
    public static final double BARAS_INIT = 0.3;

    public static final double BRAT_SUS = 0.25; // 0.85
    public static final double BRAT_JOS = 0.655; //
    public static final double CLAW_PRINS = 0.48; // 0.25 0.25
    public static final double CLAW_LASAT = 0.98; // 0.65 0.85

    public static final double BRAT_OPRIT_INT = 0.61;
    public static final double BRAT_OPRIT_EXT = 0.77;
    public static final double BRAT_OPRIT_EXT_AUTO = 0.91;
    public static final double BARA_OPRIT_INT = 0.65;
    public static final double BARA_OPRIT_EXT = 0;
    public static final double BARA_OPRIT_EXT2 = 0.07;
    public static final double BARA_OPRIT_EXT_AUTO = 0.25;

    public static final double BRAT_PARCAT = 0.93;
    public static final double BARA_PARCAT = 0.005;
}