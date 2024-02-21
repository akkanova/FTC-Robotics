package org.firstinspires.ftc.teamcode.common.misc;

import androidx.annotation.Nullable;

import java.util.function.Supplier;

/**
 * Allows the initialization of class T to be deferred
 * to a latter time when it's actually needed.
 * */
public class Lazy<T> {
    @Nullable
    private T obj;
    private final Supplier<T> supplier;

    public Lazy(Supplier<T> supplier) {
        this.supplier = supplier;
    }

    /** @return an instance of T */
    public T get() {
        if (obj == null) {
            obj = supplier.get();
        }

        return obj;
    }
}
