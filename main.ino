#include <EducationShield.h>
#include <Servo.h>
#include <CurieBLE.h>
#include <Wire.h>
#include <Adafruit_BMP280.h> 

//middle of joystick X and Y values
const int middleJoystickValue = 90;

//values of the ir sensors
int sensorLeftValue;
int sensorRightValue;

//value threshold of the sensor when it sees the black line
int blackLineThreshold = 650;

//value recieved via BLE
int recievedValue;

//BLE can only send one item at a time so it first sends X and then Y
//this flag is used to detect if X or Y is being sent
bool isXVal = true;

//flags used to flip controls and test if it is manual mode
bool flipControls = true;
bool manualMode = true;

//configuring LED PINS yellow, red, green
int ledPins[] = {10, 11, 12};

//configuring speaker pin
int piezoPin = 13;

//preset melodies
int takingSampleFrequency[] = {294, 392, 349, 494};
int takingSampleDuration[] = {100, 100, 100, 500};
int changingModesFrequency[] = {440, 330, 262};
int changingModesDuration[] = {100, 100, 500};
int flipControlFrequency[] = {294, 392, 349, 294, 494}; 
int flipControlDuration[] = {100, 100, 100, 100, 500};

//initialising pressure and temperature sensor
Adafruit_BMP280 bmp280;

//pressure and temperature values
unsigned int temperatureValue = 0;
unsigned int pressureValue = 0;

//setting x and y values to center
int xValue;
int yValue;

//initialising servos
Servo leftServo;
Servo rightServo;

//initialising BLE
BLEPeripheral blePeripheral;  

// ====  create Nordic Semiconductor UART service =========
BLEService uartService = BLEService("05642d02-009d-479a-b43e-772994edd241");

// create characteristics with UUID
BLECharacteristic drivingCharacteristic = BLECharacteristic("19b10000-e6f2-697e-4f6c-d404968a1214", BLEWriteWithoutResponse, 20);  
BLECharacteristic drivingModeCharacteristic = BLECharacteristic("74c443a0-769f-4a0f-be9a-9fe6d5d217e3", BLEWriteWithoutResponse, 20); 
BLECharacteristic takeSampleCharacteristic = BLECharacteristic("21fd85ec-a93e-48bf-94cb-38a74c0c9cff", BLEWriteWithoutResponse, 20); 
BLECharacteristic flipControlsCharacteristic = BLECharacteristic("1771433a-62e5-4722-852c-4b2048337754", BLEWriteWithoutResponse, 20); 
BLECharacteristic sendPressureTemperatureCharacteristic = BLECharacteristic("a505006c-7acc-4bc5-88b0-0a8b27ef2bb6", BLEWriteWithoutResponse, 20);
BLEUnsignedIntCharacteristic temperatureCharacteristic = BLEUnsignedIntCharacteristic("27ca9502-e6f2-697e-4f6c-d404968a1214", BLERead | BLENotify);
BLEUnsignedIntCharacteristic pressureCharacteristic2 = BLEUnsignedIntCharacteristic("043cc221-0309-404f-ae71-36c25dd21987", BLERead | BLENotify);

void setup() {
  //defining servo ports
  leftServo.attach(9);
  rightServo.attach(6);

  //defining LED pins
  pinMode(ledPins[0], OUTPUT);
  pinMode(ledPins[1], OUTPUT);
  pinMode(ledPins[2], OUTPUT);

  //defining piezo pin
  pinMode(piezoPin, OUTPUT);

  //servos are inacurate so you have to assign 92 or 93 for them to stop moving
  leftServo.write(92);
  rightServo.write(93);
   
  //initialising console output
  Serial.begin(9600);

  //initialising temperature and pressure sensor
  bmp280.begin(0x76);

  //set advertised local name and service UUID:
  blePeripheral.setLocalName("BLE_KRZYCHU");
  blePeripheral.setAdvertisedServiceUuid(uartService.uuid());

  //add characteristics:
  blePeripheral.addAttribute(uartService);

  blePeripheral.addAttribute(drivingCharacteristic);
  blePeripheral.addAttribute(drivingModeCharacteristic);
  blePeripheral.addAttribute(takeSampleCharacteristic);
  blePeripheral.addAttribute(flipControlsCharacteristic);
  blePeripheral.addAttribute(temperatureCharacteristic);
  blePeripheral.addAttribute(pressureCharacteristic2);
  blePeripheral.addAttribute(sendPressureTemperatureCharacteristic);

  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  //assign event handler for characteristics
  drivingCharacteristic.setEventHandler(BLEWritten, drivingCharacteristicWritten);
  drivingModeCharacteristic.setEventHandler(BLEWritten, drivingModeCharacteristicWritten);
  takeSampleCharacteristic.setEventHandler(BLEWritten, takeSampleCharacteristicWritten);
  flipControlsCharacteristic.setEventHandler(BLEWritten, flipButtonWritten);
  sendPressureTemperatureCharacteristic.setEventHandler(BLEWritten, sendPressureTemperatureWritten);

  //advertise the service
  blePeripheral.begin();
}

void loop() {
  //listening for events 
  blePeripheral.poll();

  //reading temperature and pressure
  temperatureValue = static_cast<unsigned int>(bmp280.readTemperature());
  pressureValue = static_cast<unsigned int>(bmp280.readPressure());
}

void blePeripheralConnectHandler(BLECentral& central) {
  //central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  manualMode = true;
  leftServo.write(92);
  rightServo.write(93);
}

void drivingModeCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) { //button that changes driving mode event handler
  //changing the flag
  manualMode = !manualMode;
  
  //stopping motors
  leftServo.write(92);
  rightServo.write(93);

  delay(500);

  //melody and led animation
  digitalWrite(ledPins[0], HIGH);
  PlayTone(piezoPin, changingModesFrequency[0], changingModesDuration[0]);
  delay(100);

  digitalWrite(ledPins[1], HIGH);
  PlayTone(piezoPin, changingModesFrequency[1], changingModesDuration[1]);
  delay(100);

  digitalWrite(ledPins[2], HIGH);
  PlayTone(piezoPin, changingModesFrequency[2], changingModesDuration[2]);
  delay(700);

  digitalWrite(ledPins[0], LOW);
  digitalWrite(ledPins[1], LOW);
  digitalWrite(ledPins[2], LOW);
}

void drivingCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) { //driving
  if (manualMode) {
    if (characteristic.value()) { //checking for null pointer errors
      recievedValue = *characteristic.value(); // * is used to dereference the pointer

      if (isXVal) { // explained in line 11
        xValue = recievedValue - middleJoystickValue;
        isXVal = !isXVal;
        return;
      }

      else {
        yValue = recievedValue - middleJoystickValue;
        isXVal = !isXVal;

        if (xValue > 2 || xValue < -2 || yValue > 2 || yValue < -2) { //joystick deadzone condition
          xValue = -xValue;

          int V = (180-Abs(xValue))*(yValue/180) + yValue;
          int W = (180-Abs(yValue))*(xValue/180) + xValue;
          int R = (W+V)/2;
          int L = (W-V)/2;

          //mapping the values 
          int tmpL = map(L, -90, 90, 0, 180);
          int tmpR = map(R, -90, 90, 0, 180);

          if (flipControls) {
            leftServo.write(tmpL);
            rightServo.write(tmpR);
          } else {
            leftServo.write(tmpR);
            rightServo.write(tmpL);
          }
        }
        else { //stops the servos if joystick is in deadzone
          leftServo.write(92);
          rightServo.write(93);
        }
        delay(20);
      }
    }
  } else { //line follower mode
    sensorRightValue = analogRead(A1); 
    sensorLeftValue = analogRead(A0);

    if((sensorRightValue < blackLineThreshold && sensorLeftValue < blackLineThreshold)) { //if both left and right sensor see the black line the robot goes forward
      //the servo is flipped so there are different values
      rightServo.write(0); 
      leftServo.write(180);
    }
    if((sensorRightValue >= blackLineThreshold && sensorLeftValue >= blackLineThreshold) || (sensorRightValue < blackLineThreshold && sensorLeftValue >= blackLineThreshold)) { //if only the left sensor sees the black line or if both sensors see the color white it turns left
      rightServo.write(180); 
      leftServo.write(180);
    }
    if(sensorLeftValue < blackLineThreshold && sensorRightValue >= blackLineThreshold) { //if only the right sensor sees the black line robot turns right
      rightServo.write(0);
      leftServo.write(0); 
    }
  }
}

int Abs(int x) { //absolute value
  if (x < 0) return -x;
  return x; 
}

void takeSampleCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) { //take sample button pressed event handler
  delay(1000);

  //melody and LED animation
  PlayTone(piezoPin, takingSampleFrequency[0], takingSampleDuration[0]);
  digitalWrite(ledPins[0], HIGH);
  delay(100);

  PlayTone(piezoPin, takingSampleFrequency[1], takingSampleDuration[1]);
  digitalWrite(ledPins[1], HIGH);
  delay(100);

  PlayTone(piezoPin, takingSampleFrequency[2], takingSampleDuration[2]);
  digitalWrite(ledPins[2], HIGH);
  delay(100);

  PlayTone(piezoPin, takingSampleFrequency[3], takingSampleDuration[3]);
  delay(1000);

  digitalWrite(ledPins[0], LOW);
  digitalWrite(ledPins[1], LOW);
  digitalWrite(ledPins[2], LOW);
}


void PlayTone(uint8_t pin, unsigned long frequency, unsigned int duration) {
	if (frequency == 0) { // If frequency is zero, just wait duration and exit.
		delay(duration);
		return;
	} 
	frequency = 1000000 / frequency; // Calculate the square wave length (in microseconds).
	uint32_t duty = frequency / 2;   // Calculate the duty cycle (volume).
	pinMode(pin, OUTPUT);            // Set pin to output mode.                        

	uint32_t startTime = millis();           // Starting time of note.
	while(millis() - startTime < duration) { // Loop for the duration.
		digitalWrite(pin,HIGH);  // Set pin high.
		delayMicroseconds(duty); // Square wave duration (how long to leave pin high).
		digitalWrite(pin,LOW);   // Set pin low.
		delayMicroseconds(frequency - duty);   // Square wave duration (how long to leave pin low).
	}
}

void flipButtonWritten(BLECentral& central, BLECharacteristic& characteristic) { //flip button pressed event handler
  //changing the flag
  flipControls = !flipControls;
  delay(200);

  //melody and led animation
  digitalWrite(ledPins[0], HIGH);
  digitalWrite(ledPins[1], HIGH);
  digitalWrite(ledPins[2], HIGH);
  delay(300);
  PlayTone(piezoPin, flipControlFrequency[0], flipControlDuration[0]);
  delay(100);

  digitalWrite(ledPins[0], HIGH);
  PlayTone(piezoPin, flipControlFrequency[1], flipControlDuration[1]);
  delay(100);

  digitalWrite(ledPins[1], HIGH);
  PlayTone(piezoPin, flipControlFrequency[2], flipControlDuration[2]);
  delay(100);

  digitalWrite(ledPins[2], HIGH);
  PlayTone(piezoPin, flipControlFrequency[3], flipControlDuration[3]);
  delay(100);
  digitalWrite(ledPins[0], LOW);
  digitalWrite(ledPins[1], LOW);
  digitalWrite(ledPins[2], LOW);

  PlayTone(piezoPin, flipControlFrequency[4], flipControlDuration[4]);
  delay(200);
  digitalWrite(ledPins[0], HIGH);
  digitalWrite(ledPins[1], HIGH);
  digitalWrite(ledPins[2], HIGH);
  delay(800);

  digitalWrite(ledPins[0], LOW);
  digitalWrite(ledPins[1], LOW);
  digitalWrite(ledPins[2], LOW);
}

void sendPressureTemperatureWritten(BLECentral& central, BLECharacteristic& characteristic) { //pressure and temperature is being sent to the phone each second
  temperatureCharacteristic.writeValue(temperatureValue);
  pressureCharacteristic2.writeValue(pressureValue);
}