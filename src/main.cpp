#include <Arduino.h>

#include <FastLED.h>
#include <DFPlayerMini_Fast.h>

//
//tweaking variables
//


//time it takes to reload when overheated
//if it's reloading without overheating
//this value is halved
int overheat_time = 5000; //in ms

//default "hp"
//this value represents how long it is until
//the pack overheats and is slowly depleted
//over time or via shooting
int proton_default_hp = 100;

//volume of the mp3 player
int pack_volume = 30; //max is 30

//these three values are used as offsets for
//the depletion of theproton hp
//see the function reduce_proton_hp()
//and run_cyclotron() for a better understanding

unsigned long default_cyclotron_time = 600; //in ms
unsigned long shooting_time = 20000; //in ms
unsigned long idle_time = 550000; //in ms

//turns on/off debugging mode
#define debugging true

//
//pins on the arduino that are in use
//

//front potentiometer is the potentiometer on the gun
//it is used to change the proton hp directly above
//a cretan threshold
#define front_potentiometer A8

//main power button on the proton pack
#define pack_power A9

//gun buttons

//power button on the gun
//lower button on the right side
#define gun_power A10

//proton indicator switch
//upper button on the right side
#define proton_indicator A11

//activate button
//upper button on the left side
#define activate A12

//button on the front of the gun
//currently set to start reload animation
#define intensify_front A13

//intensify button
//lower button on the left side
//starts the shootign animation
#define intensify A14

//output for the high power LED
//AKA the proton beam

#define mR 9
#define mG 10
#define mB 11

//PWM output for NEOPIXEL leds

#define PACK_LEDS 13
#define GUN_LEDS 12


#define PACK_NUM_LEDS 5
CRGB pack_leds[PACK_NUM_LEDS];
#define GUN_NUM_LEDS 3
CRGB gun_leds[GUN_NUM_LEDS];


//PACK LEDS names

#define cyclotron1 0
#define cyclotron2 1
#define cyclotron3 2
#define cyclotron4 3
#define N_filter 4

#define VENT_LED 0
#define White_LED 1
#define Front_LED 2

//proton_graph pins
const int proton_graph[10] = {
  22,
  23,
  24,
  25,
  26,
  27,
  28,
  29,
  30,
  31
};

//other variables for the proton_graph

#define proton_graph_max 9
int proton_graph_stage = 0;


//button variables
bool gun_power_on;
bool pack_power_on;
bool proton_indicator_on;
bool activate_on;
bool intensify_on;
bool intensify_reload;
int pot_value = 5;
bool vent_on;

//
//sound tracks
//

DFPlayerMini_Fast myMP3;


int power_down_sound = 1;
int pack_hum_sound = 2;
int gun_trail_sound = 3;
int start_up_sound = 4;
int shoot_sound = 5;
int beep_sound = 6;
int beep_shoot_sound = 7;
int gun_overheat_sound = 8;

//
//mode variables
//

#define protonAccelerator 0
#define darkMatterGenerator 1
#define plasmDistributionSystem 2
#define CompositeParticleSystem 3

//the current mode of the proton pack
//modes include:
//
//[0]protonAccelerator
//[1]darkMatterGenerator
//[2]plasmDistributionSystem
//[3]CompositeParticleSystem
int currentMode = protonAccelerator;


//
//overheating and shooting
//

unsigned long previous_overheat = 0;

//current proton hp
int proton_hp = proton_default_hp;

//current proton reduction caused by the proton indicator knob
int proton_reduction;

//previous reduction to stop the proton indicator knob from
//constantly changing
unsigned long previous_hp_reduction = 0;

//has played overheating sound?
bool beeped = false;

//is shooting?
bool shooting = false;

//last time since shooting has reduced proton hp
unsigned long last_shooting = 0;

//is the system on? if false, do startup
bool system_on = false;

//max power for the LEDS (100%)
int max_power = 100;

//state of the high power led
//makes for a smooth transition

int red_state = 0;
int green_state = 0;
int blue_state = 0;

//delay for changing high power LED
unsigned long high_power_LED_delay = 2; //in ms

//time since last led update
unsigned long previous_LED_update = 0;
//time since last color change
unsigned long previous_color_change = 0;

//should change color=
bool color_change;

//random color
long rng = 0;

//random delay between color changes
long rng_delay = 100;

//colors of the firing beam
int high_power_LED_color[4][4][3] = {
  { //protonAccelerator
    //red, yellow, white, blue
    {90, 0, 0}, {70 , 60, 0}, {90 , 80, 80}, {7 , 20, 70}
  },{ //darkMatterGenerator
    //blue, white, light blue, purple
    {0 , 0, 90}, {80 , 80, 90}, {0 , 30, 70}, {60 , 0, 80}
  },{ //plasmDistributionSystem
    //green, light green, green, white
    {0, 90, 0}, {20, 90, 10}, {0, 90, 0}, {80 , 90, 80}
  },{ //CompositeParticleSystem
    //yellow, orange, red, white
    {70 , 60, 0}, {80 , 60, 5}, {90, 0, 0}, {90 , 80, 80}
  }
};

//
//cyclotron
//

//time in between cyclotron changes
//incereeses depending on the proton hp
unsigned long cyclotron_time = default_cyclotron_time / 100 * proton_hp;

//previous cyclotron change
unsigned long cyclotron_previous_time = 0;

//previous cyclotron fade, for a smooth transition
unsigned long cyclotron_previous_fade = 0;

//fade time steps
unsigned long cyclotron_fade_time = 15; //in ms

//current cyclotron stage (1-4)
int cyclotron_stage = 0;

//is a cyclotron cell currently lighted?
bool cyclotron_on = false;

//gun lights
bool gun_lights_on;

//
//powercell
//

//time since last powercell update
unsigned long powercell_previous_time = 0; //in ms

//powercell graph pins
const int powercell[20] {
  32,
  33,
  34,
  35,
  36,
  37,
  38,
  39,
  40,
  41,
  42,
  43,
  44,
  45,
  46,
  47,
  48,
  49,
  50,
  51,
};

//0-19
#define powercell_max 19

//current powercell stage
int powercell_stage = 0;

//
//LED pulse
//

//alternate between front LED and white LED

//last pulse, for timing
unsigned long last_pulse = 0;

//which of the LEDS should be lit
bool front_LED_on = true;


//global time value to run the different timings
unsigned long t = 0;

//
//declaration of all the different functions
//

//cycle the powercell
void run_powercell();

//animation and sound when the proton pack starts
void start_up();

//turns of the gun lights
void gun_lights_off();

//reloads (sound and light animations), if overheated it lasts longer
void reload(bool overheat);

//turn on / keep running the proton pack lights
void run_proton_pack();

//shoot and depleet amunition
void shoot();

//changes brightness in the LED at the end of the gun accordingly
void high_power_LED(int LEDcolor[], unsigned long spacing_delay);

//reads the potentiometer at the front and writes a value to proton_hp
void read_potentiometer();

//reduces the proton_hp (ammonution) accordingly
void reduce_proton_hp();

//check the gun switches and act accordingly
void gun_switches();

//pulses the White_LED inversely with the Front_LED
void pulse_LED();

//runs the proton indicator graph on the proton gun
void run_proton_indicator();

//cycles to cyclotron
void run_cyclotron();

//resets variables and resets functions
void reset_pack();

//turns off cyclotron
void cyclotron_off();

//fades non-lit cyclotron cells
void fade_cyclotron();

//changes the color of the vent
void vent_color();

//changes the cyclotron color to the right one
void cyclotron_color(int currentled);

//printing debugging messages
void debugging_message();

//turning on and off switches from Serial monitor
void debugging_switches();

//
//actual code starts here
//

//the setup code is ran when the arduino starts
void setup() {
  //set pinput pins (buttons and switches)
  pinMode(pack_power, INPUT);
  pinMode(gun_power, INPUT);
  pinMode(proton_indicator, INPUT);
  pinMode(activate, INPUT);
  pinMode(intensify, INPUT);
  pinMode(intensify_front, INPUT);

  //set pinmode of proton_graph and powercell
  for (int i = 22; i <= 51; i++) {
    pinMode(i, OUTPUT);
  }
  //set output pins
  //high power leds
  pinMode(mR, OUTPUT);
  pinMode(mG, OUTPUT);
  pinMode(mB, OUTPUT);

  //neopixel leds
  FastLED.addLeds<NEOPIXEL, PACK_LEDS>(pack_leds, PACK_NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, GUN_LEDS>(gun_leds, GUN_NUM_LEDS);

  //debugging Serial
  Serial.begin(115200);

  //set up sound
  Serial1.begin(9600);
  myMP3.begin(Serial1);
  delay(500);

  pack_volume = constrain(pack_volume, 0, 30);
  myMP3.volume(pack_volume);
  FastLED.clear(true);
}


//loop while the arduino has power
void loop() {

  //read if the main power buttons are off
  #if !debugging
    gun_power_on = digitalRead(gun_power);
    pack_power_on = digitalRead(pack_power);
  #else
   debugging_switches();
   debugging_message();
  #endif

  //set time to current time
  t = millis();

  //check if the proton gun power switch is on
  if (gun_power_on) {
    run_proton_pack();
    gun_switches();
    system_on = true;
    return;

  } else if (pack_power_on) {

    run_proton_pack();

    if (gun_lights_on) {
      gun_lights_off();
    }

    system_on = true;
    return;
  } else if (system_on) {

    myMP3.play(power_down_sound);
    reset_pack();
  }

}

//turn on / keep running the proton pack lights
void run_proton_pack() {
  if (!system_on) {
    start_up();
  }
  run_cyclotron();

  run_powercell();

  reduce_proton_hp();
}

//reads the potentiometer at the front and writes a value to proton_hp
void read_potentiometer(){
  #if !debugging
    pot_value = map(analogRead(front_potentiometer), 0, 1023, 0, 5);
  #endif
  if (pot_value < 1) {
    proton_reduction = proton_default_hp - 20;
  } else {
    proton_reduction = proton_default_hp - pot_value*20;
  }
}

//animation and sound when the proton pack starts
void start_up() {
  myMP3.play(start_up_sound);
    for (int i = 0; i <= powercell_max; i++) {
        digitalWrite(powercell[i], HIGH);
    }
  for (int i = 0; i < 100; i++) {
    pack_leds[cyclotron1].setRGB( i, 0, 0);
    pack_leds[cyclotron2].setRGB( i, 0, 0);
    pack_leds[cyclotron3].setRGB( i, 0, 0);
    pack_leds[cyclotron4].setRGB( i, 0, 0);
    FastLED.show();

    delay(17);
  }

  for (int i = 0; i <= powercell_max; i++) {
    digitalWrite(powercell[i], LOW);
  }
  powercell_stage = 0;
  cyclotron_off();  
  for (int i = 0; i <= proton_graph_max; i++) {
    digitalWrite(proton_graph[i], LOW);
  }
  proton_graph_stage = 0;
  analogWrite(mR, 0);
  analogWrite(mG, 0);
  analogWrite(mB, 0);
  red_state = 0;
  green_state = 0;
  blue_state = 0;
}

//check the gun switches and act accordingly
void gun_switches() {

  gun_lights_on = true;

  //if debugging mode is on, dont read switches
  //just use the commands from the Serial monitor
  //for more see debugging_switches() and loop()
  #if !debugging
    //check the gun switches
    proton_indicator_on = digitalRead(proton_indicator);
    activate_on = digitalRead(activate);
    intensify_on = digitalRead(intensify);
    intensify_reload = digitalRead(intensify_front);
  #endif

  if (proton_indicator_on) {
    read_potentiometer();
    run_proton_indicator();

    if (activate_on) {
      pulse_LED();
      vent_color();
      FastLED.show();

      if (intensify_on) {
        shoot(); //play shooting sounds and animation whilst depleeting ammunition
      } else if (shooting) {
        myMP3.play(gun_trail_sound);
        shooting = false;
        analogWrite(mR, 0);
        analogWrite(mG, 0);
        analogWrite(mB, 0);
        red_state = 0;
        green_state = 0;
        blue_state = 0;
      }

    } else {
      //do theese if the proton indicator is on but not the generator switch
      vent_on = false;
      gun_leds[VENT_LED] = CRGB::Black;
      gun_leds[Front_LED] = CRGB::Black;
      gun_leds[White_LED] = CRGB::White;
      FastLED.show();
      front_LED_on = true;

    }
    if (intensify_reload) {
      shooting = false;
      reload(false);
      return;
    }
  } else {
    vent_on = false;
    gun_leds[VENT_LED] = CRGB::Black;
    gun_leds[Front_LED] = CRGB::Black;
    gun_leds[White_LED] = CRGB::Black;
    FastLED.show();
    proton_reduction = 0;
    for (int i = 0; i <= proton_graph_max; i++) {
    digitalWrite(proton_graph[i], LOW);
    }
    proton_graph_stage = 0;
  }
}

//cycle the powercell
void run_powercell() {
  if (t - powercell_previous_time >= cyclotron_time / 9 + 30) {
    digitalWrite(powercell[powercell_stage], HIGH);

    powercell_stage++;
    if (powercell_stage > powercell_max ) {
      for (int i = 0; i <= powercell_max; i++) {
        digitalWrite(powercell[i], LOW);
      }
      powercell_stage = 0;
    }
    
    powercell_previous_time = t;
  }
}

//cycles to cyclotron
void run_cyclotron() {
  fade_cyclotron();
  //the cyclotron time is directly relative to the proton_hp
  cyclotron_time = default_cyclotron_time / 100 * ( proton_hp - proton_reduction ) + 40;

  //while the cyclotron is on, it should stay on for a set amount of time
  if ((cyclotron_on) && (t - cyclotron_previous_time >= cyclotron_time /3*2)) {
    cyclotron_stage++;
    cyclotron_previous_time = t;
    cyclotron_on = false;
  }

  if (cyclotron_stage > 3) {
      cyclotron_stage = 0;
  }

  //if the cyclotron is off, turn it on after a time has exceeded.
  else if ((!cyclotron_on) && (t - cyclotron_previous_time >= cyclotron_time )) {
    cyclotron_color(cyclotron_stage);
    FastLED.show();

    cyclotron_on = true;
    cyclotron_previous_time = t;
  }
}

//reloads (sound and light animations), if overheated it lasts longer
void reload(bool overheat) {

  #if debugging
    Serial.println("---------!RELOADING!---------");
    if (overheat)
      Serial.println("---------!OVERHEATED!---------");
  #endif

  myMP3.play(gun_overheat_sound);

  cyclotron_off();
  for (int i = 0; i <= proton_graph_max; i++) {
    digitalWrite(proton_graph[i], LOW);
  }
  proton_graph_stage = 0;
  pack_leds[N_filter] = CRGB::Red;
  gun_leds[White_LED] = CRGB::White;
  gun_leds[Front_LED] = CRGB::Red;
  vent_color();
  FastLED.show();
  analogWrite(mR, 0);
  analogWrite(mG, 0);
  analogWrite(mB, 0);
  for (int i = 0; i <= powercell_max; i++) {
        digitalWrite(powercell[i], HIGH);
  }
  powercell_stage = 10;
  red_state = 0;
  green_state = 0;
  blue_state = 0;
  
  if (overheat) {
    delay(overheat_time);
  }
  else {
    delay(overheat_time / 2);
    intensify_reload = false;
  }
  reset_pack();
  start_up();


}

//resets variables and resets functions
void reset_pack() {
  t = millis();
  gun_lights_off();
  FastLED.clear(true);
  for (int i = 0; i <= powercell_max; i++) {
    digitalWrite(powercell[i], LOW);
  }
  powercell_stage = 0;
  FastLED.show();
  cyclotron_stage = 0;
  cyclotron_on = false;
  front_LED_on = true;
  shooting = false;
  cyclotron_previous_time = t;
  powercell_previous_time = t;

  beeped = false;
  proton_hp = proton_default_hp;
  system_on = false;
}

//reduces the proton_hp (ammonution) accordingly
void reduce_proton_hp() {

  //automaticaly reload if the proton hp is a zero
  if (proton_hp - proton_reduction <= 0) {
    reload(true);
    return;
  }
  //reduce the proton_hp slowly while the pack is ideling
  if (t - previous_hp_reduction >= idle_time / proton_default_hp) {
    proton_hp -= 1;
    previous_hp_reduction = t;
  }
  //if there is less than x hp left, play the overheat warning sound
  if ((proton_hp <= 12) && (!beeped)) {
    if (shooting) {
      myMP3.play(beep_shoot_sound);
    } else {
      myMP3.play(beep_sound);
    }

    beeped = true;   //make sure that it only beeps once
  }

}

//shoot and depleet amunition
void shoot() {
  //letting the program know it has shot so that when it turns of it can play the trail effect
  if (!shooting) {
    shooting = true;
    myMP3.play(shoot_sound);
  }

  

  if (t - previous_LED_update >= high_power_LED_delay) {
    
    if (color_change) {
      //unsigned long random_num = analogRead(RNG);
    //  randomSeed(random_num);
      rng = random(10);
      rng_delay = random(100, 300);
      color_change = false;
    }

    switch (rng)  {

      //4/10 chance of being color 1
    case 0:
    case 1:
    case 2:
    case 3:
      high_power_LED(high_power_LED_color[currentMode][0], rng_delay);
      break;

    //3/10  chance of being color 2
    case 4:
    case 5:
    case 6:
      high_power_LED(high_power_LED_color[currentMode][1], rng_delay);
      break;

    //2/10 chance of being color 3
    case 7:
    case 8:
      high_power_LED(high_power_LED_color[currentMode][2], rng_delay);
      break;

    //1/10 chance of being color 4
    case 9:
      high_power_LED(high_power_LED_color[currentMode][3], rng_delay);
      break;
    default:
    color_change = true;
      break;
    }
    previous_LED_update = t;
  }


  //reduce the proton_hp (ammonution)
  if (t - last_shooting >= shooting_time / proton_default_hp) {
    proton_hp -= 1;
    last_shooting = t;
  }

}

//changes brightness in the LED at the end of the gun accordingly
void high_power_LED(int LEDcolor[], unsigned long spacing_delay) {

  int R = constrain(LEDcolor[0], 0, max_power);
  int G = constrain(LEDcolor[1], 0, max_power);
  int B = constrain(LEDcolor[2], 0, max_power);



  if (red_state < R) {
    red_state++;
  } else if (red_state > R) {
    red_state--;
  }
  if (green_state < G) {
    green_state++;
  } else if (green_state > G) {
    green_state--;
  }
  if (blue_state < B) {
    blue_state++;
  } else if (blue_state > B) {
    blue_state--;
  }

  analogWrite(mR, red_state);
  analogWrite(mG, green_state);
  analogWrite(mB, blue_state);
  if ((red_state == R) && (green_state == G) && (blue_state == B) && (t - previous_color_change >= spacing_delay)) {
    previous_color_change = t;
    color_change = true;
  }
}

void gun_lights_off() {
  gun_lights_on = false;
  gun_leds[White_LED] = CRGB::Black;
  gun_leds[Front_LED] = CRGB::Black;
  gun_leds[VENT_LED] = CRGB::Black;
  FastLED.show();
  if (shooting) {
    delay(100);
    myMP3.play(gun_trail_sound);
    shooting = false;
  }
  analogWrite(mR, 0);
  analogWrite(mG, 0);
  analogWrite(mB, 0);
  red_state = 0;
  green_state = 0;
  blue_state = 0;
  for (int i = 0; i <= proton_graph_max; i++) {
    digitalWrite(proton_graph[i], LOW);
  }
  proton_graph_stage = 0;
}


//pulses the White_LED inversely with the Front_LED
void pulse_LED() {
  if ((t - last_pulse >= cyclotron_time / 2) && (front_LED_on)) {
    gun_leds[Front_LED] = CRGB::Black;
    front_LED_on = false;
    gun_leds[White_LED] = CRGB::White;
    last_pulse = t;
    FastLED.show();
  }
  else if ((t - last_pulse >= cyclotron_time / 2) && (!front_LED_on)) {
    gun_leds[White_LED] = CRGB::Black;
    front_LED_on = true;
    gun_leds[Front_LED] = CRGB::Red;
    last_pulse = t;
    FastLED.show();
  }
}

//runs the proton indicator graph on the proton gun
void run_proton_indicator() {
  proton_graph_stage = constrain( ( proton_hp - proton_reduction ) / 10, 0, proton_graph_max);
  
  for (int i = 0; i < proton_graph_max; i++) {
    if ( i <= proton_graph_stage) {
      digitalWrite(proton_graph[i], HIGH);
    } else {
      digitalWrite(proton_graph[i], LOW);
    }
  }
}

//changes the cyclotron color to the right one
void cyclotron_color(int currentled) {
  switch (currentMode)  {
  case protonAccelerator:
    pack_leds[currentled] = CRGB::Red;
    break;
  case darkMatterGenerator:
    pack_leds[currentled] = CRGB::Blue;
    break;
  case plasmDistributionSystem:
    pack_leds[currentled] = CRGB::Green;
    break;
  case CompositeParticleSystem:
    pack_leds[currentled] = CRGB::Orange;
    break;
  default:
    pack_leds[currentled] = CRGB::White;
  }
}

//changes the color of the vent
void vent_color() {
  vent_on = true;

  switch (currentMode)  {
  case protonAccelerator:
    gun_leds[VENT_LED] = CRGB::White;
    break;
  case darkMatterGenerator:
    gun_leds[VENT_LED] = CRGB::Blue;
    break;
  case plasmDistributionSystem:
    gun_leds[VENT_LED] = CRGB::Green;
    break;
  case CompositeParticleSystem:
    gun_leds[VENT_LED] = CRGB::Yellow;
    break;
  default:
    gun_leds[VENT_LED] = CRGB::Red;
  }
}

//turns off cyclotron
void cyclotron_off(){
  pack_leds[cyclotron1] = CRGB::Black;
  pack_leds[cyclotron2] = CRGB::Black;
  pack_leds[cyclotron3] = CRGB::Black;
  pack_leds[cyclotron4] = CRGB::Black;
  FastLED.show();
  cyclotron_stage = 3;
}

//fades non-lit cyclotron cells
void fade_cyclotron(){
  if(t - cyclotron_previous_fade < cyclotron_fade_time) {
    return;
  }

  int cyclotron_decrease = 3;

  for(int i = 0; i < 4; i++) {

    if (i == cyclotron_stage) {
      continue;
    }

    //cycle down r, g and b values
    if (pack_leds[i].r -cyclotron_decrease > 0) {
      pack_leds[i].r -= cyclotron_decrease;
    } else {
      pack_leds[i].r = 0;
    }
    if (pack_leds[i].g -cyclotron_decrease > 0) {
      pack_leds[i].g -= cyclotron_decrease;
    } else {
      pack_leds[i].g = 0;
    }
    if (pack_leds[i].b -cyclotron_decrease > 0) {
      pack_leds[i].b -= cyclotron_decrease;
    } else {
      pack_leds[i].b = 0;
    }
  }

  cyclotron_previous_fade = t;
  FastLED.show();
}

//printing debugging messages
void debugging_message() {
  
  //if debugging mode is off, don't do anything
  #if !debugging
    return;
  #endif

  delay(200);
  for (int i = 0 ; i < 30 ; i++) {
    Serial.println("");
  }
  
  if (!system_on) {
    Serial.println("System is off");
    return;
  }
  Serial.println("Switches:");
  Serial.println("gun_power_on: " + String(gun_power_on));
  Serial.println("pack_power_on: " + String(pack_power_on));
  Serial.println("proton_indicator_on: " + String(proton_indicator_on));
  Serial.println("activate_on: " + String(activate_on));
  Serial.println("intensify_on: " + String(intensify_on));
  Serial.println("intensify_reload: " + String(intensify_reload));
  Serial.println("");

  Serial.println("Proton HP: " + String(proton_hp));
  Serial.println("Effective proton HP: " + String(proton_hp - proton_reduction));
  Serial.println("t: " + String(t));
  Serial.println("Proton pack mode: " + String(currentMode));
  Serial.println();
  Serial.println("Cyclotron stage: " + String(cyclotron_stage));
  Serial.println("Powercell stage: " + String(powercell_stage));

  if (!gun_power_on) {
    return;
  }

  Serial.println("");
  Serial.println("Gun lights:");
  Serial.println("White led: " + String(!front_LED_on));
  Serial.println("front led: " + String(front_LED_on));
  Serial.println("Vent led: " + String(vent_on));
  Serial.println("Proton indicator stage: " + String(proton_graph_stage));
  Serial.println("Proton position: " + String(pot_value));
  Serial.println("Proton reduction: " + String(proton_reduction));
  Serial.println("");
  Serial.println("Shooting: " + String(shooting));

}

//turning on and off switches from Serial monitor
void debugging_switches() {
  if (Serial.available()) {
    char input = Serial.read();

    //buttons
    if (input == 'g') {
      gun_power_on = !gun_power_on;

    } else if (input == 'p') {
      pack_power_on = !pack_power_on;

    } else if (input == 'q') {
      proton_indicator_on = !proton_indicator_on;

    } else if (input == 'a') {
      activate_on = !activate_on;

    } else if (input == 'i') {
      intensify_on = !intensify_on;

    } else if (input == 'r') {
      intensify_reload = !intensify_reload;

    //potentiometer
    } else if (input == '0') {
      pot_value = 0;

    } else if (input == '1') {
      pot_value = 1;

    } else if (input == '2') {
      pot_value = 2;

    } else if (input == '3') {
      pot_value = 3;

    } else if (input == '4') {
      pot_value = 4;

    } else if (input == '5') {
      pot_value = 5;

    }
    
  }
}