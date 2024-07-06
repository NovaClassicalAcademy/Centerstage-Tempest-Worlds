package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class AutoConstants {
    @Config
    public static class Blue {
        public static double push = 3;

        //starts
        public static Pose2d BD_START = new Pose2d(18.50, 63, Math.toRadians(-90));
        public static Pose2d W_START = new Pose2d(-39.5, 63, Math.toRadians(-90));

        //spikes
        public static Pose2d BD_SPIKE_ONE = new Pose2d(33, 33.5, Math.toRadians(-135));
        public static Pose2d BD_SPIKE_TWO = new Pose2d(22, 32, Math.toRadians(-90));
        public static Pose2d BD_SPIKE_THREE = new Pose2d(10, 31, Math.toRadians(-135));

        public static Pose2d W_SPIKE_ONE = new Pose2d(-37, 35, Math.toRadians(180));
        public static Pose2d W_SPIKE_TWO = new Pose2d(-39, 12, Math.toRadians(-90));
        public static Pose2d W_SPIKE_TWO_ALT = new Pose2d(-48, 25, Math.toRadians(-180));
        public static Pose2d W_SPIKE_THREE = new Pose2d(-37.5, 18, Math.toRadians(-45));

        //stack positions
        public static Pose2d STACK_A = new Pose2d(-59, 36, Math.toRadians(180));
        public static Pose2d STACK_C = new Pose2d(-59,12,Math.toRadians(180));

        //to backdrop from spikes
        public static Pose2d W_BD_ONE_A = new Pose2d(-31, 12, Math.toRadians(180));
        public static Pose2d W_BD_ONE_B = new Pose2d(20, 12, Math.toRadians(180));

        public static Pose2d W_BD_TWO_A = new Pose2d(-31, 12, Math.toRadians(180));
        public static Pose2d W_BD_TWO_B = new Pose2d(20, 12, Math.toRadians(180));

        public static Pose2d W_BD_THREE_A = new Pose2d(-32, 12, Math.toRadians(180));
        public static Pose2d W_BD_THREE_B = new Pose2d(20, 12, Math.toRadians(180));

        //to backdrop from stacks
        public static Pose2d STACK_BD_1_A = new Pose2d(20,11, Math.toRadians(180));

        public static Pose2d STACK_BD_2_A = new Pose2d(20, 58, Math.toRadians(180));
        public static Pose2d STACK_BD_2_B = new Pose2d(-30, 58, Math.toRadians(180));

        //off backdrop
        public static Pose2d W_BD_ONE_OFF = new Pose2d(53.5, 43, Math.toRadians(180));
        public static Pose2d W_BD_TWO_OFF = new Pose2d(53.5, 36.75, Math.toRadians(180));
        public static Pose2d W_BD_THREE_OFF = new Pose2d(53.5, 31, Math.toRadians(180));

        public static Pose2d BD_BD_ONE_OFF = new Pose2d(54, 43, Math.toRadians(180));
        public static Pose2d BD_BD_TWO_OFF = new Pose2d(55, 37, Math.toRadians(180));
        public static Pose2d BD_BD_THREE_OFF = new Pose2d(54, 25.5, Math.toRadians(180));

        public static Pose2d ensuredDropL = new Pose2d(50, 43, Math.toRadians(180));
        public static Pose2d ensuredDropM = new Pose2d(50, 37, Math.toRadians(180));
        public static Pose2d ensuredDropR = new Pose2d(50, 27, Math.toRadians(180));

        //parks
        public static Pose2d PARK_CORNER = new Pose2d(52, 60, Math.toRadians(180));
        public static Pose2d PARK_CENTER = new Pose2d(55, 9, Math.toRadians(180));
    }

    @Config
    public static class Red {
        public static double push = 3;

        //starts
        public static Pose2d BD_START = new Pose2d(18.50, -63, Math.toRadians(90));
        public static Pose2d W_START = new Pose2d(-39.5, -63, Math.toRadians(90));

        //spikes
        public static Pose2d BD_SPIKE_THREE = new Pose2d(35, -33.5, Math.toRadians(135));
        public static Pose2d BD_SPIKE_TWO = new Pose2d(25, -31, Math.toRadians(135));
        public static Pose2d BD_SPIKE_ONE = new Pose2d(14, -31, Math.toRadians(180));

        public static Pose2d BD_SPIKE_ONE_extra = new Pose2d(14, -31, Math.toRadians(180));
        public static Pose2d BD_SPIKE_THREE_extra = new Pose2d(35, -33.5, Math.toRadians(135));
        public static Pose2d BD_SPIKE_TWO_extra = new Pose2d(25, -31, Math.toRadians(135));


        public static Pose2d W_SPIKE_THREE = new Pose2d(-37.25, -33.5, Math.toRadians(-180));
        public static Pose2d W_SPIKE_TWO = new Pose2d(-39, -12, Math.toRadians(90));
        public static Pose2d W_SPIKE_TWO_ALT = new Pose2d(-48, -32, Math.toRadians(-180));
        public static Pose2d W_SPIKE_ONE = new Pose2d(-37, -31, Math.toRadians(45));


        //stack positions
        public static Pose2d STACK_A = new Pose2d(-56, -37, Math.toRadians(-180));

        public static Pose2d STACK_C_1 = new Pose2d(-59.5, -12, Math.toRadians(-180));
        public static Pose2d STACK_C_2 = new Pose2d(-60, -12, Math.toRadians(-180));
        public static Pose2d STACK_C_3 = new Pose2d(-59,-12,Math.toRadians(-180));

        //to backdrop from spikes
        public static Pose2d W_BD_ONE_A = new Pose2d(-31, -12, Math.toRadians(-180));
        public static Pose2d W_BD_ONE_B = new Pose2d(20, -12, Math.toRadians(-180));

        public static Pose2d W_BD_TWO_A = new Pose2d(-31, -12, Math.toRadians(-180));
        public static Pose2d W_BD_TWO_B = new Pose2d(20, -12, Math.toRadians(-180));

        public static Pose2d W_BD_THREE_A = new Pose2d(-32, -12, Math.toRadians(-180));
        public static Pose2d W_BD_THREE_B = new Pose2d(20, -12, Math.toRadians(-180));

        //to backdrop from stacks
        public static Pose2d STACK_BD_1_A = new Pose2d(20,-11, Math.toRadians(-180));

        public static Pose2d STACK_BD_2_A = new Pose2d(20, -61, Math.toRadians(-180));
        public static Pose2d STACK_BD_2_B = new Pose2d(-38, -61, Math.toRadians(-180));

        //off backdrop
        public static Pose2d W_BD_THREE_OFF = new Pose2d(52.5, -42.5, Math.toRadians(-180));
        public static Pose2d W_BD_TWO_THREE_OFF = new Pose2d(52.5, -35.5, Math.toRadians(-180));
        public static Pose2d W_BD_TWO_TWO_OFF = new Pose2d(52.5, -38, Math.toRadians(-180));
        public static Pose2d W_BD_TWO_ONE_OFF = new Pose2d(52.5, -35.5, Math.toRadians(-180));

        public static Pose2d W_BD_ONE_THREE_OFF = new Pose2d(52.5, -29, Math.toRadians(-180));
        public static Pose2d W_BD_ONE_TWO_OFF = new Pose2d(52.5, -32, Math.toRadians(-180));
        public static Pose2d W_BD_ONE_ONE_OFF = new Pose2d(52.5, -29.5, Math.toRadians(-180));

        public static Pose2d BD_BD_THREE_OFF = new Pose2d(53, -43, Math.toRadians(-180));
        public static Pose2d BD_BD_TWO_OFF = new Pose2d(55, -37, Math.toRadians(-180));
        public static Pose2d BD_BD_ONE_OFF = new Pose2d(54, -27, Math.toRadians(-180));
        public static Pose2d BD_BD_ONE_OFF_FORWARDL = new Pose2d(51, -27, Math.toRadians(-180));
        public static Pose2d BD_BD_ONE_OFF_FORWARDM = new Pose2d(51, -35, Math.toRadians(-180));
        public static Pose2d BD_BD_ONE_OFF_FORWARDR = new Pose2d(51, -43, Math.toRadians(-180));
        //parks
        public static Pose2d PARK_CORNER = new Pose2d(52, -60, Math.toRadians(-180));
        public static Pose2d PARK_CENTER = new Pose2d(55, -9, Math.toRadians(-180));
    }

}
