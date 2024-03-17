package org.firstinspires.ftc.teamcode.common.misc;

import androidx.annotation.Nullable;

import java.util.function.Supplier;

/**
 * Allows the initialization of class T to be deferred.
 * */
public class Lazy<T> {
    @Nullable
    private T obj;
    private final Supplier<T> supplier;

    /**
     * @param supplier provides the initialization of class T.
     *                 Only called at once, when needed.
     * */
    public Lazy(Supplier<T> supplier) {
        this.supplier = supplier;
    }

    /** @return an instance of T */
    public synchronized T get() {
        if (obj == null) {
            obj = supplier.get();
        }

        return obj;
    }

    public synchronized boolean isInitialized() {
        return obj != null;
    }
}