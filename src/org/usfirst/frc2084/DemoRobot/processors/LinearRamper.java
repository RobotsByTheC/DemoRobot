/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.DemoRobot.processors;

import org.usfirst.frc2084.CMonster2014.drive.processors.ValueProcessor;

/**
 * Ramps a value by a certain amount per second. If the input increases at a
 * rate that is faster than the ramp rate, it will be reduced to ramp at the
 * correct rate. The ramper can be selected to ramp up, down or both.
 * 
 * @author Ben Wolsieffer
 */
public class LinearRamper implements ValueProcessor {

    public static class Type {

        public static final int UP_VAL = 1;
        public static final int DOWN_VAL = 2;
        public static final int UP_DOWN_VAL = 3;

        public static final Type UP = new Type(UP_VAL);
        public static final Type DOWN = new Type(DOWN_VAL);
        public static final Type UP_DOWN = new Type(UP_DOWN_VAL);

        private final int val;

        private Type(int val) {
            this.val = val;
        }

        public int value() {
            return val;
        }
    }

    private final Type type;
    private final double rampRate;
    private double lastValue;
    private double lastTime;

    public LinearRamper(double rampRate, Type type) {
        this.rampRate = Math.abs(rampRate);
        this.type = type;
        reset();
    }

    public double process(double value) {
        double currTime = Utils.getTime();
        double elapsedTime = currTime - lastTime;
        lastTime = currTime;

        double delta = value - lastValue;
        // Maximum change in the value between the previous and current
        double maxDelta = rampRate * elapsedTime;
        double output = value;
        // If the value increased too fast, limit it
        if (Math.abs(delta) > maxDelta) {
            delta = maxDelta * (delta < 0 ? -1 : 1);
            value = lastValue + delta;
            if (delta < 0) {
                if (output < 0) {
                    if (type == Type.UP) {
                        output = value;
                    }
                } else {
                    if (type == Type.DOWN) {
                        output = value;
                    }
                }
            } else {
                if (output < 0) {
                    if (type == Type.DOWN) {
                        output = value;
                    }
                } else {
                    if (type == Type.UP) {
                        output = value;
                    }
                }
            }
        }
        lastValue = output;
        return output;
    }

    public void reset() {
        lastValue = 0;
        lastTime = Utils.getTime();
    }
}
