static const int TEST_PIN = 13;

static const int D5_PIN  = 5;
static const int D6_PIN  = 6;
static const int D8_PIN  = 8;
static const int D9_PIN  = 9;
static const int D10_PIN = 10;
static const int D11_PIN = 11;
static const int D14_PIN = 14;
static const int D15_PIN = 15;

/* standby blinking and landing lights are optional features.
   They can be set by jumpers on these pins */
static const int STANDBY_MODE_ENABLE_PIN  = 4;
static const int LANDING_LIGHT_ENABLE_PIN = A3;

/* To enter standby mode or activate landing lights,
   apply an RC PWM signal to these pinns */
// Only these two pins support real interrupts on the pro mini
static const int STANDBY_MODE_CHANNEL_PIN  = 2;
static const int LANDING_LIGHT_CHANNEL_PIN = 3;

/* These thresholds define when a mode should be active */
static const uint64_t STANDBY_MODE_THRESHOLD  = 1500;
static const uint64_t LANDING_LIGHT_THRESHOLD = 1500;

/* standby and landing are read on startup */
static bool standby_mode_enabled  = false;
static bool landing_light_enabled = false;

/* Counter for main loop,
   subdivides each cycle into MILLISECONDS_PER_CYCLE steps*/
static uint64_t counter = 0;

/* The counter runs 0..COUNTER_MAX */
static const uint64_t COUNTER_MAX = 1000;

/* loop wait time MILLISECONDS_PER_STEP is such that one entire counter cycle
   takes MILLISECONDS_PER_CYCLE milliseconds */
static const uint64_t MILLISECONDS_PER_CYCLE = 5000;
static const uint64_t MILLISECONDS_PER_STEP  = MILLISECONDS_PER_CYCLE / COUNTER_MAX;

/* If standby_mode_enabled:
       if throttle is above threshold, enter FLYING mode
   else:
       always at FLYING mode
   If landing_light_enabled:
       if landing_light_channel is above threshold, turn on lights
   else:
       landing lights always off
*/
typedef enum { FLYING, STANDBY } flying_state_t;

typedef struct {
    flying_state_t fly_state;
    /* Flag indicates if landing light should be on */
    bool landing_light_active;
} state_t;

static state_t current_state = { STANDBY, false };

/* Variables to measure pulse duration of RC PWM pulse
   volatile because shared with interrupts */
static volatile uint64_t standby_channel_rise;
static volatile uint64_t standby_channel_pulse_time;
static volatile uint64_t landing_channel_rise;
static volatile uint64_t landing_channel_pulse_time;

void setup() {
    Serial.begin(115200);

    pinMode(TEST_PIN, OUTPUT);

    pinMode(D5_PIN,  OUTPUT);
    pinMode(D6_PIN,  OUTPUT);
    pinMode(D8_PIN,  OUTPUT);
    pinMode(D9_PIN,  OUTPUT);
    pinMode(D10_PIN, OUTPUT);
    pinMode(D11_PIN, OUTPUT);
    pinMode(D14_PIN, OUTPUT);
    pinMode(D15_PIN, OUTPUT);

    /* Turn off the lights */
    digitalWrite(TEST_PIN, LOW);

    digitalWrite(D5_PIN,  LOW);
    digitalWrite(D6_PIN,  LOW);
    digitalWrite(D8_PIN,  LOW);
    digitalWrite(D9_PIN,  LOW);
    digitalWrite(D10_PIN, LOW);
    digitalWrite(D11_PIN, LOW);
    digitalWrite(D14_PIN, LOW);
    digitalWrite(D15_PIN, LOW);

    pinMode(STANDBY_MODE_ENABLE_PIN,  INPUT);
    pinMode(LANDING_LIGHT_ENABLE_PIN, INPUT);

    pinMode(STANDBY_MODE_CHANNEL_PIN,  INPUT);
    pinMode(LANDING_LIGHT_CHANNEL_PIN, INPUT);

    /* Read if jumpers present (active low) */
    standby_mode_enabled  = digitalRead(STANDBY_MODE_ENABLE_PIN) == LOW;
    landing_light_enabled = digitalRead(LANDING_LIGHT_ENABLE_PIN) == LOW;

    /* attach interrupts if standby or landing light enabled */
    if (standby_mode_enabled) {
        attachInterrupt(digitalPinToInterrupt(STANDBY_MODE_CHANNEL_PIN), standby_channel_interrupt, CHANGE);
    } else {
        /* If mode not enabled, state is always FLYING */
        current_state.fly_state = FLYING;
    }
    if (landing_light_enabled) {
        attachInterrupt(digitalPinToInterrupt(LANDING_LIGHT_CHANNEL_PIN), landing_channel_interrupt, CHANGE);
    }
}

void loop() {
    if (standby_mode_enabled) {
        if (standby_channel_pulse_time < STANDBY_MODE_THRESHOLD) {
            current_state.fly_state = STANDBY;
        } else {
            current_state.fly_state = FLYING;
        }
    } else {
        current_state.fly_state = FLYING;
    }

    if (landing_light_enabled) {
        current_state.landing_light_active =
            landing_channel_pulse_time < LANDING_LIGHT_THRESHOLD;
    } else {
        current_state.landing_light_active = false;
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

static void update_test_pin(uint64_t tick, state_t state) {
    switch(state.fly_state) {
        case FLYING:
            switch(tick) {
                case 0 ... 500:
                    digitalWrite(TEST_PIN, LOW);
                    break;
                default:
                    digitalWrite(TEST_PIN, HIGH);
                    break;
            }
            break;
        case STANDBY:
            switch(tick) {
                case 0 ... 100:
                    digitalWrite(TEST_PIN, LOW);
                    break;
                case 101 ... 200:
                    digitalWrite(TEST_PIN, HIGH);
                    break;
                case 201 ... 300:
                    digitalWrite(TEST_PIN, LOW);
                    break;
                default:
                    digitalWrite(TEST_PIN, HIGH);
                    break;
            }
            break;
    }
}

static void updateOutput_D5(uint64_t tick, state_t state) {
    if (state.landing_light_active) {
        digitalWrite(D5_PIN, HIGH);
    } else {
        digitalWrite(D5_PIN, LOW);
    }
}

static void updateOutput_D6(uint64_t tick, state_t state) {
    digitalWrite(D6_PIN, HIGH);
}

static void updateOutput_D8(uint64_t tick, state_t state) {
    digitalWrite(D8_PIN, HIGH);
}

static void updateOutput_D9(uint64_t tick, state_t state) {
    digitalWrite(D9_PIN, HIGH);
}

static void updateOutput_D10(uint64_t tick, state_t state) {
    digitalWrite(D10_PIN, HIGH);
}

static void updateOutput_D11(uint64_t tick, state_t state) {
    digitalWrite(D11_PIN, HIGH);
}

static void updateOutput_D14(uint64_t tick, state_t state) {
    digitalWrite(D14_PIN, HIGH);
}

static void updateOutput_D15(uint64_t tick, state_t state) {
    digitalWrite(D15_PIN, HIGH);
}

static void standby_channel_interrupt() {
    if (digitalRead(STANDBY_MODE_CHANNEL_PIN) == HIGH) {
        standby_channel_rise = micros();
    } else {
        standby_channel_pulse_time = micros() - standby_channel_rise;
    }
}

static void landing_channel_interrupt() {
    if (digitalRead(LANDING_LIGHT_CHANNEL_PIN) == HIGH) {
        landing_channel_rise = micros();
    } else {
        landing_channel_pulse_time = micros() - landing_channel_rise;
    }
}

