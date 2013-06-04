package edu.cmu.ri.airboat.server;

import android.content.Context;
import android.util.AttributeSet;
import android.widget.ToggleButton;

public class ReadOnlyToggleButton extends ToggleButton {

    public ReadOnlyToggleButton(Context context) {
        super(context);
    }

    public ReadOnlyToggleButton(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public ReadOnlyToggleButton(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }

    @Override
    public void toggle() {
        // do nothing
    }
}