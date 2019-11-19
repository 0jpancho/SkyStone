package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {

        public final static double NORMAL_SPEED = .8;
        public final static double SLOW_SPEED = .35;

        public static final double wheelDiameter = 4;

        public static final double  wheelCircumference = wheelDiameter * Math.PI;

        public static final double gearRatio = 1/3;
        public static final double ppr = 1120;

        public final static double INTAKE_SPEED = .75;
        public final static double OUTTAKE_SPEED = .5;

        public final static double PIVOT_MULTIPLIER = .65;

        public final static double pi = Math.PI;

        public final static PIDFCoefficients teleopDrive = new PIDFCoefficients(0.1 , 0, 0, 0);
        public final static PIDFCoefficients autoTurn = new PIDFCoefficients(0.1, 0, 0, 0);
        public final static PIDFCoefficients autoDrive = new PIDFCoefficients(0.1, 0, 0, 0);

        public static double countsPerInch(){
                return (Constants.ppr *
                        Constants.gearRatio /
                        Constants.wheelCircumference);
        }

}
