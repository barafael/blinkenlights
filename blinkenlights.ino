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

static const int BUZZER_PIN = 12;

/* To enter standby mode, activate landing lights, or enable brakes etc.
   apply an RC PWM signal to these pins */
// Only 2, 3 pins support real interrupts on the pro mini
static const int STANDBY_MODE_CHANNEL_PIN  = 2;
static const int LANDING_LIGHT_CHANNEL_PIN = 3;
static const int BRAKE_CHANNEL_PIN         = A2;

/* These thresholds define when a mode should be active
   (compared against RC PWM value on resp. pin) */
static const uint64_t STANDBY_MODE_THRESHOLD  = 1500;
static const uint64_t LANDING_LIGHT_THRESHOLD = 1500;
static const uint64_t BRAKES_THRESHOLD        = 1500;

/* Counter for main loop,
   subdivides each cycle into MILLISECONDS_PER_CYCLE steps*/
static uint64_t counter = 0;

/* The counter runs 0..COUNTER_MAX */
static const uint64_t COUNTER_MAX = 1000;

/* loop wait time MILLISECONDS_PER_STEP is such that one entire counter cycle
   takes MILLISECONDS_PER_CYCLE milliseconds */
static const uint64_t MILLISECONDS_PER_CYCLE = 3000;
static const uint64_t MILLISECONDS_PER_STEP  = MILLISECONDS_PER_CYCLE / COUNTER_MAX;

typedef enum { FLYING, STANDBY } mode_t;

typedef struct {
    mode_t mode;
    /* Flag indicates if landing light should be on */
    bool landing_light_active;
    bool brake_active;
} state_t;

static state_t current_state = { STANDBY, false, false };

/* Time in microseconds after which an RC PWM signal is considered stale */
#define PWM_STALE_TIMEOUT_MICROS 4000

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

/* Non-volatile, non-shared variables to access pulse time */
static uint64_t standby_channel_pulse_time = NO_SIGNAL;
static uint64_t landing_channel_pulse_time = NO_SIGNAL;
static uint64_t brake_channel_pulse_time   = NO_SIGNAL;

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

    D5_PIN.low();
    D6_PIN.low();
    D8_PIN.low();
    D9_PIN.low();
    D10_PIN.low();
    D11_PIN.low();
    D14_PIN.low();
    D15_PIN.low();

    pinMode(STANDBY_MODE_CHANNEL_PIN,  INPUT);
    pinMode(LANDING_LIGHT_CHANNEL_PIN, INPUT);
    pinMode(BRAKE_CHANNEL_PIN,         INPUT);

    /* Attach interrupts for pulse time measurement */
    attachInterrupt(digitalPinToInterrupt(STANDBY_MODE_CHANNEL_PIN),  standby_channel_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LANDING_LIGHT_CHANNEL_PIN), landing_channel_interrupt, CHANGE);

    /* Use PinChangeInterrupt for pins other than 2, 3 */
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BRAKE_CHANNEL_PIN), brake_channel_interrupt, CHANGE);

    /* Startup buzz */
    tone(BUZZER_PIN, 880);
    delay(200);
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, 1320);
    delay(80);
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, 880);
    delay(80);
    noTone(BUZZER_PIN);
}

void loop() {
    uint64_t loop_start_millis = millis();
    /* Copy over shared volatile variables */
    noInterrupts();
    standby_channel_pulse_time = standby_channel_pulse_time_shared;
    landing_channel_pulse_time = landing_channel_pulse_time_shared;
    brake_channel_pulse_time   = brake_channel_pulse_time_shared;
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
    bool standby_mode_enabled  = standby_channel_pulse_time != NO_SIGNAL;
    bool landing_light_enabled = landing_channel_pulse_time != NO_SIGNAL;

    /* Set mode to standby only if mode is enabled */
    if (standby_mode_enabled) {
        if (standby_channel_pulse_time < STANDBY_MODE_THRESHOLD) {
            current_state.mode = STANDBY;
        } else {
            current_state.mode = FLYING;
        }
    } else {
        /* If mode not enabled, state is FLYING */
        current_state.mode = FLYING;
    }

    if (landing_light_enabled) {
        current_state.landing_light_active = landing_channel_pulse_time < LANDING_LIGHT_THRESHOLD;
    } else {
        current_state.landing_light_active = false;
    }

    if (brake_channel_pulse_time > 1500) {
        current_state.brake_active = true;
    } else {
        current_state.brake_active = false;
    }

    update_test_pin(counter, &current_state);

    updateOutput_D5(counter,  &current_state);
    updateOutput_D6(counter,  &current_state);
    updateOutput_D8(counter,  &current_state);
    updateOutput_D9(counter,  &current_state);
    updateOutput_D10(counter, &current_state);
    updateOutput_D11(counter, &current_state);
    updateOutput_D14(counter, &current_state);
    updateOutput_D15(counter, &current_state);

    /* Check for stale pulse time readings */
    uint64_t now = micros();
    if ((now - standby_channel_rise) > PWM_STALE_TIMEOUT_MICROS) {
        // ignoring interrupt here, assumption is that it won't been triggered
        // since that has not happened for PWM_STALE_TIMEOUT_MICROS microseconds
        standby_channel_pulse_time        = NO_SIGNAL;
        standby_channel_pulse_time_shared = NO_SIGNAL;
    }
    if ((now - landing_channel_rise) > PWM_STALE_TIMEOUT_MICROS) {
        // ignoring interrupt here, assumption is that it won't been triggered
        // since that has not happened for PWM_STALE_TIMEOUT_MICROS microseconds
        landing_channel_pulse_time        = NO_SIGNAL;
        landing_channel_pulse_time_shared = NO_SIGNAL;
    }
    if ((now - brake_channel_rise) > PWM_STALE_TIMEOUT_MICROS) {
        // ignoring interrupt here, assumption is that it won't been triggered
        // since that has not happened for PWM_STALE_TIMEOUT_MICROS microseconds
        brake_channel_pulse_time        = NO_SIGNAL;
        brake_channel_pulse_time_shared = NO_SIGNAL;
    }

    /* Advance counter */
    if (counter < COUNTER_MAX) {
        counter++;
    } else {
        counter = 0;
    }

//#define DEBUG_PRINT
//#define DEBUG_GRAPH
#ifdef DEBUG_PRINT
    Serial.print("Counter: ");
    Serial.print((long) counter);
    Serial.print("\t");
    Serial.print("Standby channel: ");
    Serial.print((long) standby_channel_pulse_time);
    Serial.print("\t");
    Serial.print("Landing channel: ");
    Serial.print((long) landing_channel_pulse_time);
    Serial.print("\t");
    Serial.print("Brake channel: ");
    Serial.print((long) brake_channel_pulse_time);
    Serial.println("");
#else
#ifdef DEBUG_GRAPH
    Serial.print((long) brake_channel_pulse_time);
    Serial.print("\t");
    Serial.println((long) standby_channel_pulse_time);
#endif
#endif

    /* Block until MILLISECONDS_PER_STEP have passed */
    while ((millis() - loop_start_millis) < MILLISECONDS_PER_STEP);
}

/* A macro to statically compute a fraction of the steps for a complete cycle.
 * By computing the fraction this way, a blink pattern code is independent of the
 * number of steps per cycle. */
#define cycle_fraction(a, b) (int) (((double) (a) / (double) (b)) * COUNTER_MAX)

static void update_test_pin(uint64_t tick, state_t *state) {
    switch (state->mode) {
        case FLYING:
            switch (tick) {
                case 0: TEST_PIN.high(); break;
                case cycle_fraction(4, 10): TEST_PIN.low();  break;
                case cycle_fraction(5, 10): TEST_PIN.high(); break;
                case cycle_fraction(9, 10): TEST_PIN.low();  break;
            }
            break;
        case STANDBY:
            switch (tick) {
                case 0: TEST_PIN.high(); break ;
                case cycle_fraction(1, 10):    TEST_PIN.low();  break;
                case cycle_fraction(2, 10):    TEST_PIN.high(); break;
                case cycle_fraction(3, 10):    TEST_PIN.low();  break;
                case cycle_fraction(9, 10):    TEST_PIN.high(); break;
                case cycle_fraction(9.25, 10): TEST_PIN.low();  break ;
            }
            break;
    }
}

static void updateOutput_D5(uint64_t tick, state_t *state) {
    if (state->landing_light_active) {
        D5_PIN.high();
    } else {
        D5_PIN.low();
    }
}

static void updateOutput_D6(uint64_t tick, state_t *state) {
    if (state->mode == FLYING) {
        switch (tick) {
            case 0: D6_PIN.high(); break;
            case cycle_fraction(1, 70):   D6_PIN.low();  break;
            case cycle_fraction(2, 70):   D6_PIN.high(); break;
            case cycle_fraction(3, 70):   D6_PIN.low();  break;
            case cycle_fraction(4, 70):   D6_PIN.high(); break;
            case cycle_fraction(5, 70):   D6_PIN.low();  break;
            case cycle_fraction(6, 70):   D6_PIN.high(); break;
            case cycle_fraction(7, 70):   D6_PIN.low();  break;
            case cycle_fraction(8, 70):   D6_PIN.high(); break;
            case cycle_fraction(9, 70):   D6_PIN.low();  break;
            case cycle_fraction(10, 70):  D6_PIN.high(); break;
            case cycle_fraction(11, 70):  D6_PIN.low();  break;
            case cycle_fraction(5, 10):   D6_PIN.high(); break;
            case cycle_fraction(7.5, 10): D6_PIN.low();  break;
        }
    } else if (state->mode == STANDBY) {
        if (tick < 500) {
            D6_PIN.high();
        } else {
            D6_PIN.low();
        }
    }
}

static void updateOutput_D8(uint64_t tick, state_t *state) {
    switch (tick) {
        case 0: D8_PIN.high(); break;
        case cycle_fraction(3, 10): D8_PIN.low();  break;
        case cycle_fraction(6, 10): D8_PIN.high(); break;
        case cycle_fraction(9, 10): D8_PIN.low();  break;
    }
}

static void updateOutput_D9(uint64_t tick, state_t *state) {
    switch (tick) {
        case 0: D9_PIN.low(); break;
        case cycle_fraction(3, 10): D9_PIN.high(); break;
        case cycle_fraction(6, 10): D9_PIN.low();  break;
        case cycle_fraction(9, 10): D9_PIN.high(); break;
    }
}

static void updateOutput_D10(uint64_t tick, state_t *state) {
    switch (tick) {
        case 0: D10_PIN.high(); break;
        case cycle_fraction(3, 10): D10_PIN.low();  break;
        case cycle_fraction(6, 10): D10_PIN.high(); break;
        case cycle_fraction(9, 10): D10_PIN.low();  break;
    }
}

static void updateOutput_D11(uint64_t tick, state_t *state) {
    switch (tick) {
        case 0: D11_PIN.low(); break;
        case cycle_fraction(3, 10): D11_PIN.high(); break;
        case cycle_fraction(6, 10): D11_PIN.low();  break;
        case cycle_fraction(9, 10): D11_PIN.high(); break;
    }
}

static void updateOutput_D14(uint64_t tick, state_t *state) {
    switch (tick) {
        case 0: D14_PIN.high(); break;
        case cycle_fraction(3, 10): D14_PIN.low();  break;
        case cycle_fraction(6, 10): D14_PIN.high(); break;
        case cycle_fraction(9, 10): D14_PIN.low();  break;
    }
}

static void updateOutput_D15(uint64_t tick, state_t *state) {
    switch (tick) {
        case 0: D15_PIN.low(); break;
        case cycle_fraction(3, 10): D15_PIN.high(); break;
        case cycle_fraction(6, 10): D15_PIN.low();  break;
        case cycle_fraction(9, 10): D15_PIN.high(); break;
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
    if (trigger == RISING) {
        brake_channel_rise = micros();
    } else if (trigger == FALLING) {
        brake_channel_pulse_time_shared = micros() - brake_channel_rise;
    }
}
