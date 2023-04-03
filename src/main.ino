#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

#define INDICATOR_PIN (17)
#define INPUT_PIN_INT2 (39)
#define INPUT_PIN_INT1 (37)
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

/*
volatile int32_t overruns = 0;
volatile int32_t activity = 0;
void int1_isr(void){
	activity++;
}

volatile int32_t samples = 0;
volatile int32_t inactivity = 0;
void int2_isr(void){
  inactivity ++;
}
*/

/** Global variable to determine which interrupt(s) are enabled on the ADXL343. */
int_config g_int_config_enabled = { 0 };

/** Global variables to determine which INT pin interrupt(s) are mapped to on the ADXL343. */
int_config g_int_config_map = { 0 };

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch(accel.getDataRate())
  {
    case ADXL343_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}



void setup(void){
  Serial.begin(115200);

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 detected ... Check your wiring!");
    while(1);
  }

  /* Map specific interrupts to one of the two INT pins. */
  g_int_config_map.bits.overrun    = ADXL343_INT1;
  g_int_config_map.bits.watermark  = ADXL343_INT2;
  g_int_config_map.bits.freefall   = ADXL343_INT1;
  g_int_config_map.bits.inactivity = ADXL343_INT2;
  g_int_config_map.bits.activity   = ADXL343_INT1;
  g_int_config_map.bits.double_tap = ADXL343_INT1;
  g_int_config_map.bits.single_tap = ADXL343_INT1;
  g_int_config_map.bits.data_ready = ADXL343_INT1;
  accel.mapInterrupts(g_int_config_map);

	/* Enable interrupts on the accelerometer. */
  g_int_config_enabled.bits.overrun    = false;    /* Set the INT1 */
  g_int_config_enabled.bits.watermark  = false;
  g_int_config_enabled.bits.freefall   = false;
  g_int_config_enabled.bits.inactivity = true;
  g_int_config_enabled.bits.activity   = true;
  g_int_config_enabled.bits.double_tap = false;
  g_int_config_enabled.bits.single_tap = false;
  g_int_config_enabled.bits.data_ready = false;
  accel.enableInterrupts(g_int_config_enabled);


  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_16_G);
	//accel.setDataRate(ADXL343_DATARATE_25_HZ); //ADXL343_DATARATE_0_10_HZ); //ADXL343_DATARATE_3200_HZ);

  /* Display some basic information on this sensor */
  accel.printSensorDetails();
  displayDataRate();
  Serial.println("");

  //See https://www.analog.com/media/en/technical-documentation/data-sheets/adxl343.pdf register 0x38
  // Set FIFO mode, watermark of 16 samples
  //accel.writeRegister(ADXL3XX_REG_FIFO_CTL, 0b01010000); 

  //uint8_t rval = accel.readRegister(ADXL3XX_REG_FIFO_CTL);
  //Serial.print("FIFO CTL 0x");
  //Serial.println(rval, HEX);

  // Set activity thresholds for active and inactive
  accel.writeRegister(ADXL3XX_REG_THRESH_ACT, 0x01);
  accel.writeRegister(ADXL3XX_REG_THRESH_INACT, 0x01);

  accel.writeRegister(ADXL3XX_REG_ACT_INACT_CTL, 0b11111111); //AC coupled, enable all axes active/inactive
  accel.writeRegister(ADXL3XX_REG_TIME_INACT, 0b1); // inactivity time 1 second

	pinMode(INDICATOR_PIN, OUTPUT);

  pinMode(INPUT_PIN_INT1, INPUT);
  //attachInterrupt(digitalPinToInterrupt(INPUT_PIN_INT1), int1_isr, RISING);

  pinMode(INPUT_PIN_INT2, INPUT);
  //attachInterrupt(digitalPinToInterrupt(INPUT_PIN_INT2), int2_isr, RISING);

}


int confidence = 0;
void loop(void)
{
  delay(10);

  if (digitalRead(INPUT_PIN_INT1)) {
    confidence = min(20, confidence+1);
  } else {
    confidence = max(0, confidence-1);
  }
  
  digitalWrite(INDICATOR_PIN, (confidence > 10) ? HIGH : LOW);
  accel.checkInterrupts();
}
