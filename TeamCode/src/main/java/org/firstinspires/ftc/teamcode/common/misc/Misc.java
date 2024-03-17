package org.firstinspires.ftc.teamcode.common.misc;

import java.util.function.Consumer;

/**
 * A class containing random assortments of helper methods.
 * They don't have any intentional common theme,
 * aside from being generic helpers.
 * */
public final class Misc {
    /** Checks if the nullable object is not null, then runs the provided callback */
    public static <T> void doIfNotNull(T nullableObject, Consumer<T> callback) {
        if (nullableObject != null)
            callback.accept(nullableObject);
    }

    /**
     * y = x^2, plot in Desmos. Where x is your gamepad stick input and y
     *  is your target power
     *  */
    public static double easeWithSquare(double value) {
        double squared = value * value;
        if (value < 0)
            squared *= -1;
        return squared;
    }
}
