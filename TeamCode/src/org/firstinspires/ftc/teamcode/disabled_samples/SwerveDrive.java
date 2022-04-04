package org.firstinspires.ftc.teamcode.disabled_samples;

import android.graphics.PointF;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class SwerveDrive {

    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public static final double TICKS_PER_ROTATION = 1120;
    public static final double MAX_TICKS_PER_SECOND = 2500;

    public static final double CHASSIS_RAD_SQUARED = 7*7 + 8*8;
    public static final double MAX_DRIVE_SPEED = MAX_TICKS_PER_SECOND * WHEEL_CIRCUMFERENCE / TICKS_PER_ROTATION;
    public static final double MAX_ANGULAR_SPEED = MAX_DRIVE_SPEED / Math.sqrt(CHASSIS_RAD_SQUARED);

    CRServo[] crServos = new CRServo[4];    //Steering
    DcMotor[] encoders = new DcMotor[4];    //Encoders to monitor steering
    DcMotor[] motors = new DcMotor[4];

    //Positions of the four drive wheels in robot-coordinate system, in inches
    public static final PointF[] WHEEL_POS = new PointF[]{
            new PointF(-8, -7),           //Back Left
            new PointF(-8, 7),            //Front Left
            new PointF(8, 7),             //Front Right
            new PointF(8, -7)             //Back Right
    };


    public void init(HardwareMap hardwareMap) {
        String[] crServoNames = new String[]{"back_left_crservo", "front_left_crservo", "front_right_crservo", "back_right_crservo"};
        for (int i = 0; i < 4; i++)
            crServos[i] = (CRServo) hardwareMap.get(CRServo.class, crServoNames[i]);
        String[] encoderNames = new String[]{"back_left_encoder", "front_left_encoder", "front_right_encoder", "back_right_encoder"};
        for (int i = 0; i < 4; i++)
            encoders[i] = (DcMotor) hardwareMap.get(DcMotor.class, encoderNames[i]);
        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (int i = 0; i < 4; i++)
            motors[i] = (DcMotor) hardwareMap.get(DcMotor.class, motorNames[i]);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setDriveSpeed(float vx, float vy, float va, double xz,double z) {
        boolean xd = vx==0&&vy==0&&va==0;
        double[] wheelPowers = new double[4];
        if(xd){
            for (int i = 0; i < 4; i++) {
                crServos[i].setPower(0);
                motors[i].setPower(wheelPowers[i]);
            }
        }
        else if(va==0) {
            for (int i = 0; i < 4; i++) {
                double targetSteer = (normalizeRadians(Math.atan2(vy, vx) - Math.PI / 2))-xz;
                boolean reversed = setSteer(i, targetSteer);
                wheelPowers[i] = Math.sqrt(vx * vx + vy * vy);
                if (reversed) {

                    wheelPowers[i] *= -1;
                }
                motors[i].setPower(wheelPowers[i]);
            }
        }
        else if((vx!=0||vy!=0)&&va!=0){
            for (int i = 0; i < 4; i++) {
                double[] wheelPowersM = new double[4];
                double[] wheelPowersR = new double[4];
                double targetSteerx = (normalizeRadians(Math.atan2(vy, vx) - Math.PI / 2)-xz)+Math.toRadians(20);

                boolean reversedx = checkReversed(i, targetSteerx);
                targetSteerx = returnAngularSpeed(i,targetSteerx);
                wheelPowersM[i] = Math.sqrt(vx * vx + vy * vy);
                if (reversedx) {
                    wheelPowersM[i] *= -1;
                }

                double targetSteerR = normalizeRadians(Math.atan2(va * WHEEL_POS[i].x, -(va * WHEEL_POS[i].y)) - Math.PI/2);
                boolean reversedR = checkReversed(i, targetSteerR);
                targetSteerR = returnAngularSpeed(i,targetSteerR);
                wheelPowersR[0] =z;
                wheelPowersR[1] =z;
                wheelPowersR[2] =z;
                wheelPowersR[3] =z;

                if (reversedR)
                    wheelPowersR[i] *= -1;
                if (z>0){
                    wheelPowersR[i] *= -1;
                }
                double targetSteer = (targetSteerR*(.9)) +(targetSteerx*(.1));
                wheelPowers[i] = (wheelPowersM[i]*(.1)) + (wheelPowersR[i]*(.9));
                motors[i].setPower(wheelPowers[i]);
                setSteerSimple(i,targetSteer);
            }
            }
            else{
            for (int i = 0; i < 4; i++) {
                double targetSteer = normalizeRadians(Math.atan2(va * WHEEL_POS[i].x, -(va * WHEEL_POS[i].y)) - Math.PI/2);
                boolean reversed = setSteer(i, targetSteer);
                wheelPowers[0] =z;
                wheelPowers[1] =z;
                wheelPowers[2] =z;
                wheelPowers[3] =z;
                if (reversed)
                    wheelPowers[i] *= -1;
                if (z>0){
                    wheelPowers[i] *= -1;
                }
                motors[i].setPower(wheelPowers[i]);
            }
            }
    }

    private double getSteerRadians(int i){
        return normalizeRadians(2.0 * Math.PI * encoders[i].getCurrentPosition()/TICKS_PER_ROTATION);
    }
    private boolean setSteer(int i, double targetSteer){
        double currentSteer = getSteerRadians(i);
        double offset = normalizeRadians(targetSteer - currentSteer);
        boolean result = false;
        if (Math.abs(offset) > Math.PI/2){
            result = true;
            offset = normalizeRadians(offset + Math.PI);
        }
        double steerPower = Range.clip(4.0*offset/Math.PI, -1, 1);
        crServos[i].setPower(steerPower);
        return result;
    }
    private void setSteerSimple(int i, double targetSteer){
        crServos[i].setPower(targetSteer);
    }
    private boolean checkReversed(int i, double targetSteer){
        double currentSteer = getSteerRadians(i);
        double offset = normalizeRadians(targetSteer - currentSteer);
        boolean result = false;
        if (Math.abs(offset) > Math.PI/2){
            result = true;
            offset = normalizeRadians(offset + Math.PI);
        }
        double steerPower = Range.clip(4.0*offset/Math.PI, -1, 1);
        return result;
    }
    private double returnAngularSpeed(int i, double targetSteer){
        double currentSteer = getSteerRadians(i);
        double offset = normalizeRadians(targetSteer - currentSteer);
        boolean result = false;
        if (Math.abs(offset) > Math.PI/2){
            result = true;
            offset = normalizeRadians(offset + Math.PI);
        }
        double steerPower = Range.clip(4.0*offset/Math.PI, -1, 1);
        return steerPower;
    }
    public static double normalizeRadians(double radians) {
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }


}
