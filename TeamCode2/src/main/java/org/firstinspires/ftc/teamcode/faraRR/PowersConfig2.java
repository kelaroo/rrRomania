package org.firstinspires.ftc.teamcode.faraRR;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PowersConfig2 {

    public static final double COEFF_SPEED_HIGH = 1;
    public static final double COEFF_SPEED_LOW = 0.7;

    public static final double INTAKE_SUCK = 0.587;//1; //0.85;
    public static final double INTAKE_AUTO_B = 1;


    public static final PIDFCoefficients lansatCoeff = new PIDFCoefficients(25, 0, 15.5, 7);
    public static final double LANSAT_SPEED = 1635;
    public static final double LANSAT_SPEED_PS = 1490 ;
    public static final double LANSAT_POWER = 0.66; //0.62;
    public static final double LANSAT_POWER_PS = 0.575;
    public static final double LANSAT_POWER_AUTO = 0.65;

    public static final double LANSAT_AUTO_PS = 1415; //1425
    public static final double LANSAT_AUTO_A1 = 1625;
    public static final double LANSAT_AUTO_B1 = 1630;
    public static final double LANSAT_AUTO_B2 = 1585;
    public static final double LANSAT_AUTO_C1 = 1600;
    public static final double LANSAT_AUTO_A_PS = 1455;
    public static final double LANSAT_AUTO_B_PS = 1480;
    public static final double LANSAT_AUTO_C_PS = 1470;

    public static final double INTAKE2_RIGHT = 1;
    public static final double INTAKE2_LEFT = 0;
    public static final double INTAKE2_STATIONARY = 0.5;

    public static final double INTAKE3_SUCK = 1;

    public static final double CUVA_SUS = 0.428;
    public static final double CUVA_JOS = 0.70;

    public static final double IMPINS_FWD = 0.5;
    public static final double IMPINS_BWD = 0.80;
    public static final double IMPINS_SECOND = 0.88;

    public static final double BRAT_SUS = 0.27;
    public static final double BRAT_JOS = 0.9;
    public static final double CLAW_PRINS = 0.325;
    public static final double CLAW_LASAT = 0.75;

    public static final double BRAT_OPRIT_INIT = 0;
    public static final double BRAT_OPRIT_SUS = 0.35;
    public static final double BRAT_OPRIT_JOS = 0.65;
    public static final double BRAT_OPRIT_C = 0.9;
    public static final double BARA_OPRIT_SUCK = 1;
    public static final double BARA_OPRIT_STATIONARY = 0.5;
    public static final double BARA_OPRIT_OUT = 0;
}
