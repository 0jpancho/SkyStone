package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {

        private static final double wheelDiameter = 4; //inches

        private static final double  wheelCircumference = wheelDiameter * Math.PI;

        private static final double gearRatio = 1/2;
        private  static final double ppr = 1120;

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
