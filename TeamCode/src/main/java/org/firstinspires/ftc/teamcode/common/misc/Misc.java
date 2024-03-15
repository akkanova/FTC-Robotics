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
}
