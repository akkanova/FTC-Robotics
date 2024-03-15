package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Objects;

/**
 * An extension of {@link Gamepad}. Inspired by
 * <a href="https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/gamepad/GamepadEx.java">
 *      FTClib's GamepadEx.java
 * </a>.
 * In a similar fashion, it does not support PS4 controllers.
 * */
public class GamepadEx {
    public enum Button {
        DPAD_UP("dpad_up"),
        DPAD_DOWN("dpad_down"),
        DPAD_LEFT("dpad_left"),
        DPAD_RIGHT("dpad_right"),
        A("a"),
        B("b"),
        X("x"),
        Y("y"),
        START("start"),
        BACK("back"),
        LEFT_STICK_BUTTON("left_stick_button"),
        RIGHT_STICK_BUTTON("right_stick_button"),
        LEFT_BUMPER("left_bumper"),
        RIGHT_BUMPER("right_bumper"),
        LEFT_TRIGGER("left_trigger", true),
        RIGHT_TRIGGER("right_trigger", true);

        public final String gamepadFieldName;
        public final boolean isFloat;

        Button(String gamepadFieldName) {
            this(gamepadFieldName, false);
        }

        Button(String gamepadFieldName, boolean isFloat) {
            this.gamepadFieldName = gamepadFieldName;
            this.isFloat = isFloat;
        }
    }

    //-----------------------------------------------------------------------------------
    // Constructor
    //-----------------------------------------------------------------------------------

    private final HashMap<Button, Runnable> onceButtonClickedListeners;
    private final HashMap<Button, Runnable> onButtonClickListeners;
    private final HashMap<Button, Runnable> onButtonUnClickListeners;
    private final HashMap<Button, ButtonState> buttonStates;
    private final float triggerThreshold;
    private final Gamepad gamepad;

    /**
     * @param gamepad the {@link Gamepad} instance.
     * @param triggerThreshold how much the trigger should be pressed
     *                         for it to be considered as a truthy value.
     * */
    public GamepadEx(Gamepad gamepad, float triggerThreshold) {
        this.onceButtonClickedListeners = new HashMap<>();
        this.onButtonClickListeners = new HashMap<>();
        this.onButtonUnClickListeners = new HashMap<>();
        this.triggerThreshold = triggerThreshold;
        this.buttonStates = new HashMap<>();
        this.gamepad = gamepad;

        for (Button button : Button.values()) {
            ButtonState state = button.isFloat
                ? new TriggerState(button)
                : new ButtonState(button);

            if (state.isValid())
                this.buttonStates.put(button, state);
        }

        this.sync();
    }

    /** Use 0.5 as default value for {@code triggerThreshold} */
    public GamepadEx(Gamepad gamepad) {
        this(gamepad, 0.5f);
    }

    /**
     * Synchronize current values with the current
     * values of target gamepad.
     * */
    public void sync() {
        for (ButtonState state : this.buttonStates.values()) {
            state.update();
            if (!state.previous && state.current) {
                getListenerAndRun(this.onceButtonClickedListeners, state.button, true);
                getListenerAndRun(this.onButtonClickListeners, state.button, false);

            } else if (state.previous && !state.current) {
                getListenerAndRun(this.onButtonUnClickListeners, state.button, false);
            }
        }
    }

    /**
     * Helper function to retrieve the listener for a button, from the provided
     * listeners map, and to also run it, if it exists.
     */
    private static void getListenerAndRun(
            HashMap<Button, Runnable> listenersMap,
            Button button,
            boolean runOnlyOnce
    ) {
        Runnable listener = listenersMap.get(button);
        if (listener != null) {
            listener.run();
            if (runOnlyOnce)
                listenersMap.remove(button);
        }
    }

    //-------------------------------------------------------------------------------
    // Getters
    //-------------------------------------------------------------------------------

    /** @return the original SDK {@link Gamepad} instance */
    public Gamepad getOriginalGamepad() {
        return this.gamepad;
    }

    /** @return {@code gamepad.left_stick_x} */
    public float getLeftX() {
        return gamepad.left_stick_x;
    }

    /** @return {@code gamepad.left_stick_y} */
    public float getLeftY() {
        return gamepad.left_stick_y;
    }

    /** @return {@code gamepad.right_stick_x} */
    public float getRightX() {
        return gamepad.right_stick_x;
    }

    /** @return {@code gamepad.right_stick_y} */
    public float getRightY() {
        return gamepad.right_stick_y;
    }

    /** @return whether that current button is pressed */
    public boolean isPressed(Button button) {
        return Objects.requireNonNull(this.buttonStates.get(button)).current;
    }

    //-------------------------------------------------------------------------------
    // Event Listeners
    //-------------------------------------------------------------------------------

    /**
     * Adds a once button clicked listener; a listener that only runs once, and
     * then immediately dispose off. If a listener already existed for that
     * button, it will overwrite it.
     * */
    public void onceButtonClicked(Button button, Runnable runnable) {
        this.onceButtonClickedListeners.put(button, runnable);
    }

    /**
     * Adds an on button click listener. Called when the buttons state goes from previously
     * not pressed to a one that is. If a listener already existed for that button,
     * it will overwrite it.
     * */
    public void onButtonClick(Button button, Runnable runnable) {
        this.onButtonClickListeners.put(button, runnable);
    }

    /**
     * Adds once button un-click / de-press listener; listens for a change of button state
     * from one that was pressed to a one that is not pressed.
     */
    public void onButtonUnClicked(Button button, Runnable runnanble) {
        this.onButtonUnClickListeners.put(button, runnanble);
    }

    /** Removes all "once" and "on" button click listeners */
    public void removeAllListeners() {
        this.onButtonClickListeners.clear();
        this.onceButtonClickedListeners.clear();
        this.onButtonUnClickListeners.clear();
    }

    //-----------------------------------------------------------------------------------
    // States
    //-----------------------------------------------------------------------------------

    /**
     * Represents the current and the saved previous value of a {@link Gamepad} button.
     * Through the use of Java's Reflection API, we can simplify the fetching of boolean
     * values from the {@link Gamepad} class' fields.
     * */
    private class ButtonState {
        protected Field gamepadField;
        protected boolean valid;

        public boolean previous = false;
        public boolean current = false;
        public final Button button;

        private ButtonState(Button button) {
            this.button = button;
            try {
                this.gamepadField = Gamepad.class.getField(button.gamepadFieldName);
                this.gamepadField.setAccessible(true);
                this.valid = !Modifier.isFinal(this.gamepadField.getModifiers())
                            && !Modifier.isStatic(this.gamepadField.getModifiers());

            } catch (NoSuchFieldException ignored) {
                this.valid = false;
            }
        }

        /**
         * If this button state is valid, or the Java Reflection API can access the field.
         * Then the "previous" value is replaced with the "current" value, and the "current" value
         * is replaced with the newest value fetched through the reflection API.
         * */
        public void update() {
            if (this.valid) {
                try {
                    this.previous = this.current;
                    this.current = gamepadField.getBoolean(gamepad);
                } catch (IllegalAccessException e) {
                    // It's not suppose to throw an error...
                    throw new RuntimeException(e);
                }
            }
        }

        /** Can the Java reflection API access this field? */
        private boolean isValid() {
            return this.valid;
        }
    }

    /**
     * Same as Button State but converts the {@link Gamepad} trigger (which is a float)
     * into a boolean.
     * */
    private class TriggerState extends ButtonState {
        private TriggerState(Button button) {
            super(button);
        }

        @Override
        public void update() {
            if (this.valid) {
                try {
                    this.previous = this.current;
                    this.current = gamepadField.getFloat(gamepad) > triggerThreshold;
                } catch (IllegalAccessException e) {
                    // It's not suppose to throw an error...
                    throw new RuntimeException(e);
                }
            }
        }
    }
}
