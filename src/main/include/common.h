#pragma once

enum SubSystemState {
    HOLD,
    MANUAL_MOVING,
    AUTO_MOVING
};

/*
double signum(double value) {
    if (value > 0.0) {
        return 1.0;
    }
    else if (value < 0.0) {
        return -1.0;
    }
    else {
        return 0.0;
    }
}
*/