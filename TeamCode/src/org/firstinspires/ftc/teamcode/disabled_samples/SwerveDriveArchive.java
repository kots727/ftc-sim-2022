package org.firstinspires.ftc.teamcode.disabled_samples;

import android.graphics.PointF;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class SwerveDriveArchive {

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

    /**
     * Sets the robot drive speed (linear and angular)
     * @param vx    Speed (inches per sec) in X direction (positive is rightward)
     * @param vy    Speed (inches per sec) in Y direction (positive is forward)
     * @param va    Angular speed (radians per sec)   (positive is counter-clockwise)
     */
    /*public void setDriveSpeed(float vx, float vy, float va, double xz){
        va = (float) (va*Math.PI*2);
        float a = vx-(va*7);
        float b = vx+(va*7);
        float c = vy-(va*8);
        float d = vy+(va*8);
        motors[0].setPower(Math.sqrt(a*a+d*d));
        motors[1].setPower(Math.sqrt(b*b+d*d));
        motors[2].setPower(Math.sqrt(b*b+c*c));
        motors[3].setPower(Math.sqrt(a*a+c*c));
        double targetSteer = (normalizeRadians(Math.atan2(vy, vx) - Math.PI / 2)) - xz;
        boolean reversed = setSteer(0,Math.atan2(a,d)*(180/Math.PI));        //True if the setSteer method has reversed the steer angle         //If steer angle reversed, must also reverse drive power
        reversed = setSteer(1,Math.atan2(b,d)*(180/Math.PI));
        reversed = setSteer(2,Math.atan2(b,c)*(180/Math.PI));
        reversed = setSteer(3,Math.atan2(a,c)*(180/Math.PI));
    }*/
    public void setDriveSpeed(float vx, float vy, float va, double xz,double z) {
        boolean xd = vx==0&&vy==0&&va==0;
        double[] wheelPowers = new double[4];

        //Determine the power needed for each wheel.
        for (int i=0; i<4; i++){
            double wheelSpeed = Math.sqrt(vx * vx + vy * vy + va * va * CHASSIS_RAD_SQUARED
                    + 2 * va * (vy * WHEEL_POS[i].x - vx * WHEEL_POS[i].y));
            wheelPowers[i] = (wheelSpeed / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION / MAX_TICKS_PER_SECOND;
        }

        //Wheel powers must be in the range -1 to +1. If they are not, then we must scale down the values
        //of each wheel power, and also the values of vx, vy, and va
/*        double max = 1.0;
        for (int i=0; i<4; i++) max = Math.max(max, Math.abs(wheelPowers[i]));
        if (max > 1.0){
            vx /= max;
            vy /= max;
            va /= max;
            for (int i=0; i<4; i++) wheelPowers[i] /= max;
        }*/

        //Set the steering for each wheel, then set the power for each wheel
        if(xd){
            for (int i = 0; i < 4; i++) {
                crServos[i].setPower(0);
                motors[i].setPower(wheelPowers[i]);
            }
        }
        else if(va==0) {
            for (int i = 0; i < 4; i++) {
                /*
                 * targetSteer is the target steering direction for the wheel, if we were going to use positive drive power.
                 * But, we could just as easily reverse that steering direction (by adding/subtracting 180 degrees), and
                 * use negative drive power instead. We do when needed so that we are never trying to turn to a steer
                 * angle that is more than 90 degrees away from our current steer angle.
                 */
                double targetSteer = (normalizeRadians(Math.atan2(vy, vx) - Math.PI / 2))-xz;
                boolean reversed = setSteer(i, targetSteer);
                wheelPowers[i] = Math.sqrt(vx * vx + vy * vy);//True if the setSteer method has reversed the steer angle
                if (reversed) {

                    wheelPowers[i] *= -1;
                }//If steer angle reversed, must also reverse drive power
                motors[i].setPower(wheelPowers[i]);
            }
        }
        else if((vx!=0||vy!=0)&&va!=0){
            for (int i = 0; i < 4; i++) {
                /*
                 * targetSteer is the target steering direction for the wheel, if we were going to use positive drive power.
                 * But, we could just as easily reverse that steering direction (by adding/subtracting 180 degrees), and
                 * use negative drive power instead. We do when needed so that we are never trying to turn to a steer
                 * angle that is more than 90 degrees away from our current steer angle.
                 */
                //double targetSteer =  normalizeRadians(Math.atan2(va * WHEEL_POS[i].x,va * WHEEL_POS[i].y) - Math.PI/2);
                double[] wheelPowersM = new double[4];
                double[] wheelPowersR = new double[4];
                double targetSteerx = (normalizeRadians(Math.atan2(vy, vx) - Math.PI / 2)-xz)+Math.toRadians(20);

                boolean reversedx = checkReversed(i, targetSteerx);
                targetSteerx = returnAngularSpeed(i,targetSteerx);
                wheelPowersM[i] = Math.sqrt(vx * vx + vy * vy);//True if the setSteer method has reversed the steer angle
                if (reversedx) {
                    wheelPowersM[i] *= -1;
                }//If steer angle reversed, must also reverse drive power

                double targetSteerR = normalizeRadians(Math.atan2(va * WHEEL_POS[i].x, -(va * WHEEL_POS[i].y)) - Math.PI/2);
                boolean reversedR = checkReversed(i, targetSteerR);
                targetSteerR = returnAngularSpeed(i,targetSteerR);
                wheelPowersR[0] =z;
                wheelPowersR[1] =z;
                wheelPowersR[2] =z;
                wheelPowersR[3] =z;//True if the setSteer method has reversed the steer angle

                if (reversedR)
                    wheelPowersR[i] *= -1;                 //If steer angle reversed, must also reverse drive power
                if (z>0){
                    wheelPowersR[i] *= -1;
                }
                double targetSteer = (targetSteerR*(.6)) +(targetSteerx*(.4));
                wheelPowers[i] = (wheelPowersM[i]*(.4)) + (wheelPowersR[i]*(.6));
                motors[i].setPower(wheelPowers[i]);
                setSteerSimple(i,targetSteer);
            }
            }
            else{
            for (int i = 0; i < 4; i++) {
                /*
                 * targetSteer is the target steering direction for the wheel, if we were going to use positive drive power.
                 * But, we could just as easily reverse that steering direction (by adding/subtracting 180 degrees), and
                 * use negative drive power instead. We do when needed so that we are never trying to turn to a steer
                 * angle that is more than 90 degrees away from our current steer angle.
                 */
                //double targetSteer =  normalizeRadians(Math.atan2(va * WHEEL_POS[i].x,va * WHEEL_POS[i].y) - Math.PI/2);
                //double targetSteer = normalizeRadians(Math.atan2(vy + va * WHEEL_POS[i].x, vx - va * WHEEL_POS[i].y) - Math.PI/2);
                double targetSteer = normalizeRadians(Math.atan2(va * WHEEL_POS[i].x, -(va * WHEEL_POS[i].y)) - Math.PI/2);
                boolean reversed = setSteer(i, targetSteer);
                wheelPowers[0] =z;
                wheelPowers[1] =z;
                wheelPowers[2] =z;
                wheelPowers[3] =z;//True if the setSteer method has reversed the steer angle
                if (reversed)
                    wheelPowers[i] *= -1;                 //If steer angle reversed, must also reverse drive power
                if (z>0){
                    wheelPowers[i] *= -1;
                }
                motors[i].setPower(wheelPowers[i]);
            }
            }
    }

    /**
     * Get the current steering angle, in radians, of the specified steering unit
     * @param i         Index of the steering unit
     * @return          Current steer angle in radians
     */
    private double getSteerRadians(int i){
        return normalizeRadians(2.0 * Math.PI * encoders[i].getCurrentPosition()/TICKS_PER_ROTATION);
    }

    /**
     * Steer an individual drive wheel (i) toward the specified target angle, using proportionate control.
     * But, if the target angle is more than 90 degrees away from the current angle, reverse the target angle
     * by adding/subtracting 180 degrees.
     *
     * @param i                 Index of the steering unit to set
     * @param targetSteer       Target steer angle in radians
     * @return                  True if steer angle was reversed, otherwise false
     */
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



    /**
     * Normalize an angle into the range of -PI to +PI radians
     * @param radians
     * @return
     */
    public static double normalizeRadians(double radians) {
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }


}
