#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <DigitalIO.h>

static DigitalPin<13> TEST_PIN;

static DigitalPin<5>  D5_PIN;
static DigitalPin<6>  D6_PIN;
static DigitalPin<8>  D8_PIN;
static DigitalPin<9>  D9_PIN;
static DigitalPin<10> D10_PIN;
static DigitalPin<11> D11_PIN;
static DigitalPin<14> D14_PIN;
static DigitalPin<15> D15_PIN;

/* standby blinking and landing lights are optional features.
   They can be set by jumpers on these pins */
static const int STANDBY_MODE_ENABLE_PIN  = 4;
static const int LANDING_LIGHT_ENABLE_PIN = A3;

/* To enter standby mode or activate landing lights,
   apply an RC PWM signal to these pinns */
// Only these two pins support real interrupts on the pro mini
static const int STANDBY_MODE_CHANNEL_PIN  = 2;
static const int LANDING_LIGHT_CHANNEL_PIN = 3;
static const int BRAKE_CHANNEL_PIN = A2;

/* These thresholds define when a mode should be active
   (compared against RC PWM value on resp. pin) */
static const uint64_t STANDBY_MODE_THRESHOLD  = 1500;
static const uint64_t LANDING_LIGHT_THRESHOLD = 1500;

/* standby mode and landing light feature enabled-ness */
static bool standby_mode_enabled  = false;
static bool landing_light_enabled = false;

/* Counter for main loop,
   subdivides each cycle into MILLISECONDS_PER_CYCLE steps*/
static uint64_t counter = 0;

/* The counter runs 0..COUNTER_MAX */
static const uint64_t COUNTER_MAX = 1000;

/* loop wait time MILLISECONDS_PER_STEP is such that one entire counter cycle
   takes MILLISECONDS_PER_CYCLE milliseconds */
static const uint64_t MILLISECONDS_PER_CYCLE = 3000;
static const uint64_t MILLISECONDS_PER_STEP  = MILLISECONDS_PER_CYCLE / COUNTER_MAX;

/* If standby_mode_enabled:
       if threshold condition is met, enter STANDBY mode
   else:
       enter FLYING mode
   If landing_light_enabled:
       if  threshold condition is met, turn on lights
   else:
       lights off
*/
typedef enum { FLYING, STANDBY } flying_state_t;

typedef struct {
    flying_state_t fly_state;
    /* Flag indicates if landing light should be on */
    bool landing_light_active;
    bool brake_active;
} state_t;

static state_t current_state = { STANDBY, false, false };

// Flag invalid RC PWM pulse signal
#define NO_SIGNAL (-1)

/* Variables to measure pulse duration of RC PWM pulse
   volatile because shared with interrupts */
static volatile uint64_t standby_channel_rise;
static volatile uint64_t standby_channel_pulse_time_shared;
static volatile uint64_t landing_channel_rise;
static volatile uint64_t landing_channel_pulse_time_shared;
static volatile uint64_t brake_channel_rise;
static volatile uint64_t brake_channel_pulse_time_shared;

static uint64_t standby_channel_pulse_time = NO_SIGNAL;
static uint64_t landing_channel_pulse_time = NO_SIGNAL;
static uint64_t brake_channel_pulse_time = NO_SIGNAL;

void setup() {
    Serial.begin(250000);

    TEST_PIN.mode(OUTPUT);

    D5_PIN.mode (OUTPUT);
    D6_PIN.mode (OUTPUT);
    D8_PIN.mode (OUTPUT);
    D9_PIN.mode (OUTPUT);
    D10_PIN.mode(OUTPUT);
    D11_PIN.mode(OUTPUT);
    D14_PIN.mode(OUTPUT);
    D15_PIN.mode(OUTPUT);

    /* Turn off the lights */
    TEST_PIN.low();

    D5_PIN. low();
    D6_PIN. low();
    D8_PIN. low();
    D9_PIN. low();
    D10_PIN.low();
    D11_PIN.low();
    D14_PIN.low();
    D15_PIN.low();

    pinMode(STANDBY_MODE_ENABLE_PIN,  INPUT);
    pinMode(LANDING_LIGHT_ENABLE_PIN, INPUT);

    pinMode(STANDBY_MODE_CHANNEL_PIN,  INPUT);
    pinMode(LANDING_LIGHT_CHANNEL_PIN, INPUT);
    pinMode(BRAKE_CHANNEL_PIN, INPUT);

    /* attach interrupts even if modes are not enabled right now */
    attachInterrupt(digitalPinToInterrupt(STANDBY_MODE_CHANNEL_PIN), standby_channel_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LANDING_LIGHT_CHANNEL_PIN), landing_channel_interrupt, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BRAKE_CHANNEL_PIN), brake_channel_interrupt, CHANGE);
}

void loop() {
    /* Copy over shared variables */
    noInterrupts();
    standby_channel_pulse_time = standby_channel_pulse_time_shared;
    landing_channel_pulse_time = landing_channel_pulse_time_shared;
    brake_channel_pulse_time    = brake_channel_pulse_time_shared;
    interrupts();

    /* Set NO_SIGNAL when there is no valid PWM pulse on the input */
    if (standby_channel_pulse_time < 500 || standby_channel_pulse_time > 3000) {
        standby_channel_pulse_time = NO_SIGNAL;
    }
    if (landing_channel_pulse_time < 500 || landing_channel_pulse_time > 3000) {
        landing_channel_pulse_time = NO_SIGNAL;
    }
    if (brake_channel_pulse_time < 500 || brake_channel_pulse_time > 3000) {
        brake_channel_pulse_time = NO_SIGNAL;
    }

    /* Read if jumpers present (active low) */
    standby_mode_enabled  = digitalRead(STANDBY_MODE_ENABLE_PIN)  == LOW;
    landing_light_enabled = digitalRead(LANDING_LIGHT_ENABLE_PIN) == LOW;

    /* Set mode to standby only if mode is enabled */
    if (standby_mode_enabled) {
        if (standby_channel_pulse_time < STANDBY_MODE_THRESHOLD) {
            current_state.fly_state = STANDBY;
        } else {
            current_state.fly_state = FLYING;
        }
    } else {
        /* If mode not enabled, state is FLYING */
        current_state.fly_state = FLYING;
    }

    if (landing_light_enabled) {
        current_state.landing_light_active =
            landing_channel_pulse_time < LANDING_LIGHT_THRESHOLD;
    } else {
        current_state.landing_light_active = false;
    }

    if (brake_channel_pulse_time > 1500) {
        current_state.brake_active = true;
    } else {
        current_state.brake_active = false;
    }

    update_test_pin (counter, current_state);

    updateOutput_D5 (counter, current_state);
    updateOutput_D6 (counter, current_state);
    updateOutput_D8 (counter, current_state);
    updateOutput_D9 (counter, current_state);
    updateOutput_D10(counter, current_state);
    updateOutput_D11(counter, current_state);
    updateOutput_D14(counter, current_state);
    updateOutput_D15(counter, current_state);

    if (counter < COUNTER_MAX) {
        counter++;
    } else {
        counter = 0;
    }

    delay(MILLISECONDS_PER_STEP);

//#define DEBUG_PRINT
#ifdef DEBUG_PRINT
    Serial.print("Counter: ");
    Serial.print((long)counter);
    Serial.print("\t");
    Serial.print("Standby channel: ");
    Serial.print((long)standby_channel_pulse_time);
    Serial.print("\t");
    Serial.print("Landing channel: ");
    Serial.print((long)landing_channel_pulse_time);
    Serial.println("\t");
#endif
}

#define cycle_fraction(a, b) \
    (int)(((double)(a)/(double)(b)) * COUNTER_MAX)

static void update_test_pin(uint64_t tick, state_t state) {
    switch(state.fly_state) {
    case FLYING:
        switch(tick) {
        case 0:
            TEST_PIN.high();
            break;
        case cycle_fraction(4, 10):
            TEST_PIN.low();
            break;
        case cycle_fraction(5, 10):
            TEST_PIN.high();
            break;
        case cycle_fraction(9, 10):
            TEST_PIN.low();
            break;
        }
        break;
    case STANDBY:
        switch(tick) {
        case 0:
            TEST_PIN.high();
            break;
        case cycle_fraction(1, 10):
            TEST_PIN.low();
            break;
        case cycle_fraction(2, 10):
            TEST_PIN.high();
            break;
        case cycle_fraction(3, 10):
            TEST_PIN.low();
            break;
        case cycle_fraction(9, 10):
            TEST_PIN.high();
            break;
        case cycle_fraction(9.25, 10):
            TEST_PIN.low();
            break;
        }
        break;
    }
}

static void updateOutput_D5(uint64_t tick, state_t state) {
    if (state.landing_light_active) {
        D5_PIN.high();
    } else {
        D5_PIN.low();
    }
}

static void updateOutput_D6(uint64_t tick, state_t state) {
    if (state.fly_state == FLYING) {
        switch (tick) {
            case 0:
                D6_PIN.high();
                break;
            case cycle_fraction(1, 70):
                D6_PIN.low();
                break;
            case cycle_fraction(2, 70):
                D6_PIN.high();
                break;
            case cycle_fraction(3, 70):
                D6_PIN.low();
                break;
            case cycle_fraction(4, 70):
                D6_PIN.high();
                break;
            case cycle_fraction(5, 70):
                D6_PIN.low();
                break;
            case cycle_fraction(6, 70):
                D6_PIN.high();
                break;
            case cycle_fraction(7, 70):
                D6_PIN.low();
                break;
            case cycle_fraction(8, 70):
                D6_PIN.high();
                break;
            case cycle_fraction(9, 70):
                D6_PIN.low();
                break;
            case cycle_fraction(10, 70):
                D6_PIN.high();
                break;
            case cycle_fraction(11, 70):
                D6_PIN.low();
                break;
            case cycle_fraction(5, 10):
                D6_PIN.high();
                break;
            case cycle_fraction(7.5, 10):
                D6_PIN.low();
                break;
        }
    } else if (state.fly_state == STANDBY) {
        if (tick < 500) {
            D6_PIN.high();
        } else {
            D6_PIN.low();
        }
    }
}

static void updateOutput_D8(uint64_t tick, state_t state) {
    switch(tick) {
        case 0:
            D8_PIN.high();
            break;
        case cycle_fraction(3, 10):
            D8_PIN.low();
            break;
        case cycle_fraction(6, 10):
            D8_PIN.high();
            break;
        case cycle_fraction(9, 10):
            D8_PIN.low();
            break;
    }
}

static void updateOutput_D9(uint64_t tick, state_t state) {
    switch(tick) {
        case 0:
            D9_PIN.low();
            break;
        case cycle_fraction(3, 10):
            D9_PIN.high();
            break;
        case cycle_fraction(6, 10):
            D9_PIN.low();
            break;
        case cycle_fraction(9, 10):
            D9_PIN.high();
            break;
    }
}

static void updateOutput_D10(uint64_t tick, state_t state) {
    switch(tick) {
        case 0:
            D10_PIN.high();
            break;
        case cycle_fraction(3, 10):
            D10_PIN.low();
            break;
        case cycle_fraction(6, 10):
            D10_PIN.high();
            break;
        case cycle_fraction(9, 10):
            D10_PIN.low();
            break;
    }
}

static void updateOutput_D11(uint64_t tick, state_t state) {
    switch(tick) {
        case 0:
            D11_PIN.low();
            break;
        case cycle_fraction(3, 10):
            D11_PIN.high();
            break;
        case cycle_fraction(6, 10):
            D11_PIN.low();
            break;
        case cycle_fraction(9, 10):
            D11_PIN.high();
            break;
    }
}

static void updateOutput_D14(uint64_t tick, state_t state) {
    switch(tick) {
        case 0:
            D14_PIN.high();
            break;
        case cycle_fraction(3, 10):
            D14_PIN.low();
            break;
        case cycle_fraction(6, 10):
            D14_PIN.high();
            break;
        case cycle_fraction(9, 10):
            D14_PIN.low();
            break;
    }
}

static void updateOutput_D15(uint64_t tick, state_t state) {
    switch(tick) {
        case 0:
            D15_PIN.low();
            break;
        case cycle_fraction(3, 10):
            D15_PIN.high();
            break;
        case cycle_fraction(6, 10):
            D15_PIN.low();
            break;
        case cycle_fraction(9, 10):
            D15_PIN.high();
            break;
    }
}

static void standby_channel_interrupt() {
    if (digitalRead(STANDBY_MODE_CHANNEL_PIN) == HIGH) {
        standby_channel_rise = micros();
    } else {
        standby_channel_pulse_time_shared = micros() - standby_channel_rise;
    }
}

static void landing_channel_interrupt() {
    if (digitalRead(LANDING_LIGHT_CHANNEL_PIN) == HIGH) {
        landing_channel_rise = micros();
    } else {
        landing_channel_pulse_time_shared = micros() - landing_channel_rise;
    }
}

static void brake_channel_interrupt() {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(BRAKE_CHANNEL_PIN));
    if(trigger == RISING) {
        brake_channel_rise = micros();
    } else if(trigger == FALLING) {
        brake_channel_pulse_time_shared = micros() - brake_channel_rise;
    }
}
