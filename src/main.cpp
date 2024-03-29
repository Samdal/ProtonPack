#include <Arduino.h>

#include <FastLED.h>
#include <DFPlayerMini_Fast.h>

// time it takes to reload when overheated
// if not overheatign reloading takes half the time
#define OVERHEAT_TIME 5000

// default "hp"
// this value represents how long it is until
// the pack overheats and is slowly depleted
// over time or via shooting
#define DEFAULT_PROTON_HP 100

#define DEFAULT_PROTON_PACK_TIME 600
#define SHOOTING_TIME 20000
#define IDLE_TIME 550000

#define DEBUGGING 0

// front potentiometer is the potentiometer on the gun
// it is used to change the proton hp directly above
// a cretan threshold
#define FRONT_POTENTIOMETER A8
#define PACK_POWER A9
// lower button on the right side
#define GUN_POWER A10
// upper button on the right side
#define PROTON_INDICATOR A11
// upper button on the left side
#define ACTIVATE A12
// button on the front of the gun
#define INTENSIFY_FRONT A13
// lower button on the left side, starts the shootign animation
#define INTENSIFY A14
// motor that vibrates the neutrino wand
#define VIBRATOR 8
// output for the high power led, AKA the proton beam
#define HP_LED_R 9
#define HP_LED_G 10
#define HP_LED_B 11

enum {
        PROTON_ACCELERATOR,
        DARK_MATTER_GENERATOR,
        PLASM_DISTRIBUTION_SYSTEM,
        COMPOSITE_PARTICLE_SYSTEM,
        PROTON_MODES_COUNT,
} proton_modes;
int current_mode = PROTON_ACCELERATOR;

// cyclotron fade time
#define CYCLOTRON_FADE_TIME 10
#define CYCLOTRON_FADE_AMOUNT 5

enum {
        CYCLOTRON1,
        CYCLOTRON2,
        CYCLOTRON3,
        CYCLOTRON4,
        N_FILTER,
        PACK_NUM_LEDS,
} pack_leds_names;
#define PACK_LEDS_PIN 13
CRGB pack_leds[PACK_NUM_LEDS];

const CRGB cyclotron_colors[PROTON_MODES_COUNT] = {
        CRGB::Red,
        CRGB::Blue,
        CRGB::Green,
        CRGB::DarkOrange,
};
#define SET_CYCLOTRON_ON(_led) pack_leds[_led] = cyclotron_colors[current_mode]

enum {
	GUN_ON_LED,
	WHITE_LED,
	VENT_LED,
	FRONT_LED,
	GUN_NUM_LEDS,
} gun_leds_names;
#define GUN_LEDS_PIN 12
CRGB gun_leds[GUN_NUM_LEDS];

const CRGB gun_led_colors[PROTON_MODES_COUNT][GUN_NUM_LEDS] = {
	{
		CRGB::Red,
        	CRGB::White,
        	CRGB::White,
		CRGB::OrangeRed,
	}, {
		CRGB::Red,
        	CRGB::White,
        	CRGB::Blue,
		CRGB::OrangeRed,
	}, {
		CRGB::Red,
        	CRGB::White,
        	CRGB::Green,
		CRGB::OrangeRed,
	}, {
		CRGB::Red,
        	CRGB::White,
        	CRGB::Yellow,
		CRGB::OrangeRed,
	},
};
#define SET_GUN_LED_ON(_led) gun_leds[_led] = gun_led_colors[current_mode][_led]

#define JRGEN_EXTRA

#ifdef JRGEN_EXTRA
enum {
        power_down_sound = 1,
        pack_hum_sound,
        gun_trail_sound,
        start_up_sound,
        beep_sound,
        beep_shoot_sound,
        gun_overheat_sound,
        shoot_sound,
	ghostbusters_theme_song,
} tracks;
#else
enum {
        power_down_sound = 1,
        pack_hum_sound,
        gun_trail_sound,
        start_up_sound,
        beep_sound,
        beep_shoot_sound,
        gun_overheat_sound,
} trakcs;
#endif

#define PACK_VOLUME 30
DFPlayerMini_Fast myMP3;

#define PROTON_GRAPH_COUNT 10
const uint8_t proton_graph[PROTON_GRAPH_COUNT] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
uint8_t proton_graph_stage = 0;

#define POWERCELL_COUNT 20
const int powercell[POWERCELL_COUNT] {32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};
uint8_t powercell_stage = 0;

// button variables (stored here so that debug mode can change them)
bool pack_power_on = true;
bool gun_power_on;
bool proton_indicator_on;
bool activate_on;
bool intensify_on;
bool intensify_reload;

bool front_led_on = true;
bool vent_led_on = true;
bool shooting;
bool system_on;

// current proton reduction caused by the proton indicator knob
int proton_reduction;
uint8_t proton_indicator_pot_value = 5;

uint8_t cyclotron_stage;

uint8_t proton_hp = DEFAULT_PROTON_HP;

// state of the high power led
// for smooth transition
int red_state = 0;
int green_state = 0;
int blue_state = 0;

// colors of the firing beam
uint8_t high_power_led_colors[PROTON_MODES_COUNT][4][3] = {
        { // PROTON_ACCELERATOR
                // red-orange, blue, yellow, white
                {255, 130, 0}, {20 , 20, 255}, {255 , 180, 0}, {255, 200, 230},
        },
        { // DARK_MATTER_GENERATOR
                // blue, white, light blue, purple
                {0 , 0, 90}, {80 , 80, 90}, {0 , 30, 70}, {60 , 0, 80}
        },
        { // PLASM_DISTRIBUTION_SYSTEM
                // green, light green, green, white
                {0, 90, 0}, {20, 90, 10}, {0, 90, 0}, {80 , 90, 80}
        },
        { // COMPOSITE_PARTICLE_SYSTEM
                // yellow, orange, red, white
                {70 , 60, 0}, {80 , 60, 5}, {90, 0, 0}, {90 , 80, 80}
        }
};


// time in between cyclotron changes
// incereeses depending on the proton hp
unsigned long proton_pack_time = DEFAULT_PROTON_PACK_TIME / 100 * proton_hp;

// global time value to run the different timings
unsigned long t = 0;

void start_up();
void run_proton_pack();
void reduce_proton_hp();
void run_proton_gun();
void gun_lights_off();
void shoot();
void reload(bool overheat);
void reset_pack();

void run_powercell();
void powercell_off();

// changes brightness in the led at the end of the gun accordingly
bool high_power_led(const uint8_t* ledcolor, const unsigned long spacing_delay, bool no_fade);
void high_power_led_off();

bool start_vent(bool reset = false);
void pulse_led(); // pulses the WHITE_LED inversely with the FRONT_LED
void run_proton_indicator();
void read_potentiometer();
void proton_indicator_off();

void run_cyclotron();
void fade_cyclotron();
void cyclotron_off();

void debugging_message();
void debugging_switches();

void setup()
{
#if DEBUGGING
        Serial.begin(115200);
        Serial.println("Debug mode entered");
        delay(1000);
#endif

        // input pins (buttons and switches)
        pinMode(PACK_POWER,       INPUT);
        pinMode(GUN_POWER,        INPUT);
        pinMode(PROTON_INDICATOR, INPUT);
        pinMode(ACTIVATE,         INPUT);
        pinMode(INTENSIFY,        INPUT);
        pinMode(INTENSIFY_FRONT,  INPUT);

        // output pins
        for (int i = 0; i < PROTON_GRAPH_COUNT; i++)
                pinMode(proton_graph[i], OUTPUT);
        for (int i = 0; i < POWERCELL_COUNT; i++)
                pinMode(powercell[i], OUTPUT);

        pinMode(HP_LED_R, OUTPUT);
        pinMode(HP_LED_G, OUTPUT);
        pinMode(HP_LED_B, OUTPUT);
        pinMode(VIBRATOR, OUTPUT);

        // neopixel leds
        FastLED.addLeds<NEOPIXEL, PACK_LEDS_PIN>(pack_leds, PACK_NUM_LEDS);
        FastLED.addLeds<NEOPIXEL, GUN_LEDS_PIN>(gun_leds, GUN_NUM_LEDS);
        FastLED.clear(true);
        cyclotron_off();
        gun_lights_off();

        // sound
        Serial1.begin(9600);
        myMP3.begin(Serial1);
        delay(100);
}


// loop while the arduino has power
void loop()
{
#if DEBUGGING
        debugging_switches();
        //debugging_message();
#else
        gun_power_on = digitalRead(GUN_POWER);
#endif
        t = millis();

        if (pack_power_on || gun_power_on) {
                if (!system_on) {
                        start_up();
                        system_on = true;
                }

                if (gun_power_on) run_proton_gun();
                else              gun_lights_off();

                run_proton_pack();
        }
        if (!pack_power_on && !gun_power_on && system_on) {
                myMP3.play(power_down_sound);
                reset_pack();
                system_on = false;
                delay(100);
        }
}

// turn on / keep running the proton pack lights
void run_proton_pack()
{
        proton_pack_time = DEFAULT_PROTON_PACK_TIME / DEFAULT_PROTON_HP * ( proton_hp - proton_reduction ) + 40;

        run_cyclotron();
        run_powercell();
        reduce_proton_hp();
	FastLED.show();
}

// reads the potentiometer at the front and writes a value to proton_hp
void read_potentiometer()
{
#if !DEBUGGING
        proton_indicator_pot_value = map(analogRead(FRONT_POTENTIOMETER), 0, 1023, 0, 5);
#endif
        if (proton_indicator_pot_value == 5) proton_indicator_pot_value = 4;

        if (proton_indicator_pot_value == 0) proton_reduction = DEFAULT_PROTON_HP - 20;
        else               proton_reduction = DEFAULT_PROTON_HP - (proton_indicator_pot_value + 1) * 20;
}

// animation and sound when the proton pack starts
void start_up()
{
        myMP3.play(start_up_sound);

        for (int i = 0; i < POWERCELL_COUNT; i++)
                digitalWrite(powercell[i], HIGH);

	// TODO make it fade towards selected mdoes target color
        for (int i = 0; i < 255; i++) {
                pack_leds[CYCLOTRON1].setRGB(i, 0, 0);
                pack_leds[CYCLOTRON2].setRGB(i, 0, 0);
                pack_leds[CYCLOTRON3].setRGB(i, 0, 0);
                pack_leds[CYCLOTRON4].setRGB(i, 0, 0);
                FastLED.show();

                delay(10);
        }
        reset_pack();
}

// check the gun switches and act accordingly
void run_proton_gun()
{
	SET_GUN_LED_ON(GUN_ON_LED);
        // if debugging mode is on, dont read switches
        // the buttons are set from the serial monitor
        // see debugging_switches()
	static bool last_intensify_reload_state;
	last_intensify_reload_state = intensify_reload;
#if !DEBUGGING
        // check the gun switches
        proton_indicator_on = digitalRead(PROTON_INDICATOR);
        activate_on         = digitalRead(ACTIVATE);
        intensify_on        = digitalRead(INTENSIFY);
        intensify_reload    = digitalRead(INTENSIFY_FRONT);
#endif

        if (proton_indicator_on) {
                read_potentiometer();
                run_proton_indicator();

                if (activate_on) {
                        pulse_led();
			if(vent_led_on) vent_led_on = start_vent();

			static unsigned long last_shot;
                        if (intensify_on) {
				if (t - last_shot > 500)
                                	shoot();
                        } else if (shooting) {
				last_shot = t;
                                myMP3.play(gun_trail_sound);
                                shooting = false;
                                high_power_led_off();
				digitalWrite(VIBRATOR, LOW);
                        }
			if (intensify_reload) {
				shooting = false;
				reload(false);
				return;
			}
                } else {
                        // do theese if the proton indicator is on but not the generator switch
                        front_led_on        = true;
                        gun_leds[VENT_LED]  = CRGB::Black;
			vent_led_on         = true;
                        gun_leds[FRONT_LED] = CRGB::Black;
			SET_GUN_LED_ON(WHITE_LED);

			if (intensify_reload && !last_intensify_reload_state) {
				static bool is_on = false;
				is_on = !is_on;
#ifdef JRGEN_EXTRA
				if (is_on)
					myMP3.play(ghostbusters_theme_song);
				else
					myMP3.stop();
#endif
			}
                }
        } else {
                gun_leds[VENT_LED]  = CRGB::Black;
                gun_leds[FRONT_LED] = CRGB::Black;
                gun_leds[WHITE_LED] = CRGB::Black;
                proton_reduction = 0;
		vent_led_on     = true;
                proton_indicator_off();
        }
}

void powercell_off()
{
        for (int i = 0; i < POWERCELL_COUNT; i++)
                digitalWrite(powercell[i], LOW);
        powercell_stage = 0;
}

void run_powercell()
{
        static unsigned long powercell_previous_time;

        if (t - powercell_previous_time >= proton_pack_time / 6) {
                if (powercell_stage >= POWERCELL_COUNT) {
                        for (int i = 0; i < POWERCELL_COUNT; i++)
                                digitalWrite(powercell[i], LOW);
                        powercell_stage = 0;
                } else {
                        digitalWrite(powercell[powercell_stage++], HIGH);
                }
    
                powercell_previous_time = t;
        }
}

void run_cyclotron()
{
        static unsigned long cyclotron_previous_time;
        static bool cyclotron_on;

        fade_cyclotron();

        // while the cyclotron is on, it should stay on for a set amount of time
        if (cyclotron_on && t - cyclotron_previous_time >= proton_pack_time / 3 * 2) {
                cyclotron_stage++;
                cyclotron_previous_time = t;
                cyclotron_on = false;
        }

        // loop around
        if (cyclotron_stage > 3) cyclotron_stage = 0;

        // if the cyclotron is off, turn it on after a time has exceeded.
        if (!cyclotron_on && t - cyclotron_previous_time >= proton_pack_time) {
		SET_CYCLOTRON_ON(cyclotron_stage);
                cyclotron_previous_time = t;
                cyclotron_on = true;
        }
}

// reloads (sound and light animations), if overheated it lasts longer
void reload(bool overheat)
{
#if DEBUGGING
        Serial.println("---------!RELOADING!---------");
        if (overheat) Serial.println("---------!OVERHEATED!---------");
#endif
        myMP3.play(gun_overheat_sound);

	SET_CYCLOTRON_ON(CYCLOTRON1);
	SET_CYCLOTRON_ON(CYCLOTRON2);
	SET_CYCLOTRON_ON(CYCLOTRON3);
	SET_CYCLOTRON_ON(CYCLOTRON4);
        proton_indicator_off();
        for (int i = 0; i < POWERCELL_COUNT; i++)
                digitalWrite(powercell[i], HIGH);
        high_power_led_off();
	digitalWrite(VIBRATOR, LOW);

        pack_leds[N_FILTER] = CRGB::White;

	SET_GUN_LED_ON(WHITE_LED);
	SET_GUN_LED_ON(FRONT_LED);
	SET_GUN_LED_ON(VENT_LED);
        FastLED.show();
        if (overheat) {
                delay(OVERHEAT_TIME);
        } else {
                delay(OVERHEAT_TIME / 2);
                intensify_reload = false;
        }
        reset_pack();
        start_up();
}

// resets variables and resets functions
void reset_pack()
{
        gun_lights_off();
        FastLED.clear(true);
        FastLED.show();

        proton_hp       = DEFAULT_PROTON_HP;
        cyclotron_off();
        powercell_off();
}

// reduces the proton_hp (ammonution) accordingly
void reduce_proton_hp()
{
        static unsigned long previous_hp_reduction;
        static bool beeped;

        // automaticaly reload if the proton hp is a zero
        if (proton_hp - proton_reduction <= 0) {
                reload(true);
                return;
        }

        // reduce the proton_hp slowly while the pack is ideling (does not apply if there isn't any proton reduction)
        if (proton_reduction != 0 && t - previous_hp_reduction >= IDLE_TIME / DEFAULT_PROTON_HP) {
                proton_hp -= 1;
                previous_hp_reduction = t;
        }

        // if there is less than x hp left, play the overheat warning sound
        if (proton_hp - proton_reduction <= 10 && !beeped) {
                beeped = true;
                if (shooting) myMP3.play(beep_shoot_sound);
                else         myMP3.play(beep_sound);

                for (uint8_t blinkamount = 0; blinkamount <= 5; blinkamount++) {
                        for (int i = 0; i < PROTON_GRAPH_COUNT; i++)
                                digitalWrite(proton_graph[i], HIGH);
                        for (int i = 0; i < POWERCELL_COUNT; i++)
                                digitalWrite(powercell[i], HIGH);

                        cyclotron_off();
                        pack_leds[N_FILTER] = CRGB::White;
                        gun_leds[FRONT_LED] = CRGB::Black;
                        gun_leds[WHITE_LED] = CRGB::Black;
                        gun_leds[VENT_LED] = CRGB::Black;
                        FastLED.show();

						unsigned long time_now = millis();
						while (millis() - time_now < 200) {
							run_proton_gun();
						}

						proton_indicator_off();
						powercell_off();
						SET_CYCLOTRON_ON(CYCLOTRON1);
						SET_CYCLOTRON_ON(CYCLOTRON2);
						SET_CYCLOTRON_ON(CYCLOTRON3);
						SET_CYCLOTRON_ON(CYCLOTRON4);
						pack_leds[N_FILTER] = CRGB::Black;
						SET_GUN_LED_ON(VENT_LED);
						SET_GUN_LED_ON(FRONT_LED);
						SET_GUN_LED_ON(WHITE_LED);
						FastLED.show();

						time_now = millis();
						while (millis() - time_now < 200) {
							run_proton_gun();
						}
                }
        } else if (proton_hp - proton_reduction > 12 && beeped){
                beeped = false;
        }
}

// shoot and depleet amunition
void shoot()
{
        // letting the program know it has shot so that when it turns of it can play the trail effect
        if (!shooting) {
                shooting = true;
                myMP3.play(shoot_sound);
		digitalWrite(VIBRATOR, HIGH);
        }

	static bool color_change;
	static unsigned long rng_delay = 100;
	static int rng;
	static bool no_fade;

	if (color_change) {
		static bool no_fade_next;
		rng = random(10);
		rng_delay = random(80, 200);
		if (no_fade_next) {
			no_fade_next = false;
			no_fade = false;
		} else {
			if (no_fade) no_fade_next = true;
			else         no_fade = random(2) && rng_delay < 150;
		}
		color_change = false;
	}

	switch (rng)  {
	case 0:
	case 1:
	case 2:
	case 3:
		color_change = high_power_led(high_power_led_colors[current_mode][0], rng_delay + 60, no_fade);
		break;
	case 4:
	case 5:
	case 6:
		color_change = high_power_led(high_power_led_colors[current_mode][1], rng_delay, no_fade);
		break;
	case 7:
	case 8:
		color_change = high_power_led(high_power_led_colors[current_mode][2], rng_delay, no_fade);
		break;
	case 9:
		color_change = high_power_led(high_power_led_colors[current_mode][3], rng_delay, no_fade);
		break;
	}

        // reduce the proton_hp (ammonution)
        static unsigned long last_shooting;
        if (t - last_shooting >= SHOOTING_TIME / DEFAULT_PROTON_HP) {
                proton_hp -= 1;
                last_shooting = t;
        }
}

#define red_max 90
// changes brightness in the led at the end of the gun accordingly
bool high_power_led(const uint8_t* ledcolor, const unsigned long spacing_delay, bool no_fade)
{
        static unsigned long previous_color_change;
        int R = min(ledcolor[0], red_max);
        int G = ledcolor[1];
        int B = ledcolor[2];
	if (no_fade) {
		red_state = R;
		green_state = G;
		blue_state = B;
	}

        if (red_state < R)      red_state++;
        else if (red_state > R) red_state--;

#ifdef JRGEN_EXTRA
        if (green_state < G)      green_state++;
        else if (green_state > G) green_state--;
#else
	green_state = G;
#endif

        if (blue_state < B)       blue_state++;
        else if (blue_state > B)  blue_state--;

        analogWrite(HP_LED_R, red_state);
        analogWrite(HP_LED_G, green_state);
        analogWrite(HP_LED_B, blue_state);
        if (t - previous_color_change >= spacing_delay) {
                previous_color_change = t;
                return true;
        }
        return false;
}

void high_power_led_off()
{
        analogWrite(HP_LED_R, 0);
        analogWrite(HP_LED_G, 0);
        analogWrite(HP_LED_B, 0);
        red_state   = 0;
        green_state = 0;
        blue_state  = 0;
}

void gun_lights_off()
{
        proton_reduction = 0;
        front_led_on     = true;
	vent_led_on      = true;
	start_vent(true);

        gun_leds[GUN_ON_LED] = CRGB::Black;
        gun_leds[WHITE_LED]  = CRGB::Black; 
	gun_leds[FRONT_LED]  = CRGB::Black;
        gun_leds[VENT_LED]   = CRGB::Black;
        FastLED.show();

        high_power_led_off();
	digitalWrite(VIBRATOR, LOW);
        proton_indicator_off();

        if (shooting) {
                delay(100);
                myMP3.play(gun_trail_sound);
                shooting = false;
        }
}


// pulses the WHITE_LED inversely with the FRONT_LED
void pulse_led()
{
        static unsigned long previous_pulse;
        if (t - previous_pulse >= proton_pack_time) {
                if (front_led_on) {
			SET_GUN_LED_ON(WHITE_LED);
                        gun_leds[FRONT_LED] = CRGB::Black;
                } else {
                        gun_leds[WHITE_LED] = CRGB::Black;
			SET_GUN_LED_ON(FRONT_LED);
                }
                front_led_on = !front_led_on;
                previous_pulse   = t;
        }
}

#define vent_blink_time 60
bool start_vent(bool reset) {
	static unsigned long last_vent_blink;
	static int vent_times;
	static bool vent_on;

	if (vent_times > 4 || reset) {
		if (!reset) SET_GUN_LED_ON(VENT_LED);
		vent_times = 0;
		return false;
	}
	if (millis() - last_vent_blink < vent_blink_time)
		return true;

	if (vent_on)
        	gun_leds[VENT_LED] = CRGB::Black;
	else
		SET_GUN_LED_ON(VENT_LED);

	vent_on = !vent_on;
	last_vent_blink = millis();
	vent_times++;
	return true;
}

void run_proton_indicator()
{
        proton_graph_stage = (proton_hp - proton_reduction) / (DEFAULT_PROTON_HP / 10);
        for (int i = 0; i < PROTON_GRAPH_COUNT; i++) {
                if (i <= proton_graph_stage) digitalWrite(proton_graph[i], HIGH);
                else                         digitalWrite(proton_graph[i], LOW);
        }
}

void proton_indicator_off()
{
        for (int i = 0; i < PROTON_GRAPH_COUNT; i++)
                digitalWrite(proton_graph[i], LOW);
}

void cyclotron_off()
{
        pack_leds[CYCLOTRON1] = CRGB::Black;
        pack_leds[CYCLOTRON2] = CRGB::Black;
        pack_leds[CYCLOTRON3] = CRGB::Black;
        pack_leds[CYCLOTRON4] = CRGB::Black;
        FastLED.show();
}

void fade_cyclotron()
{
        static unsigned long cyclotron_previous_fade;
        if(t - cyclotron_previous_fade < CYCLOTRON_FADE_TIME) return;

        for(int i = 0; i < 4; i++) {
                if (i == cyclotron_stage) continue;

                // cycle down r, g and b values
                for(int c = 0; c < 3; c++)
                        if (pack_leds[i].raw[c] >= CYCLOTRON_FADE_AMOUNT)
                                pack_leds[i].raw[c] -= CYCLOTRON_FADE_AMOUNT;
                        else
                                pack_leds[i].raw[c] = 0;
        }
        cyclotron_previous_fade = t;
}

// turning on and off switches from Serial monitor
void debugging_switches()
{
        if (Serial.available()) {
                switch(Serial.read()) {
                case 'g': gun_power_on        = !gun_power_on;         break;
                case 'p': pack_power_on       = !pack_power_on;        break;
                case 'q': proton_indicator_on = !proton_indicator_on;  break;
                case 'a': activate_on         = !activate_on;          break;
                case 'i': intensify_on        = !intensify_on;         break;
                case 'r': intensify_reload    = !intensify_reload;     break;

                case '0': proton_indicator_pot_value = 0; break;
                case '1': proton_indicator_pot_value = 1; break;
                case '2': proton_indicator_pot_value = 2; break;
                case '3': proton_indicator_pot_value = 3; break;
                case '4': proton_indicator_pot_value = 4; break;
                case '5': proton_indicator_pot_value = 5; break;
                }
        }
}
