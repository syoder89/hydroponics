#include "Adafruit_INA219.h"
#include "Adafruit_AM2320.h"

#define SENSOR_ID "ScottHydro"
Adafruit_INA219 ina219;
Adafruit_INA219 ina219_b(0x41);
Adafruit_AM2320 am2320 = Adafruit_AM2320();

SYSTEM_MODE(SEMI_AUTOMATIC);

int pump = D8; // Instead of writing D0 over and over again, we'll write pump
Timer pumpStateTimer(60*1000, evaluatePumpState);
Timer pumpOffTimer(5*60*1000, pumpOff);
Timer pumpOnTimer(5*60*1000, pumpOn);
Timer publishTimer(5*60*1000, schedulePublish);
int pumpRunTime;
int pumpOffTime;
boolean pumpRunning = false;
double rawtemp = 0.0, rawhumidity = 0.0, solarCurrent = 0.0;
double batteryCurrent = 0.0;
double solarPower = 0.0, batteryPower = 0.0, totalPower = 0.0;
double solarVoltage = 0.0, batteryVoltage = 0.0;
int uptime;
char data[256];
boolean doPublish = false;

void setup() {
	int i;
	Serial.begin(115200);
	pinMode(pump, OUTPUT);
	ina219.begin();
	ina219_b.begin();
	am2320.begin();
	Particle.variable("solarCurrent", solarCurrent);
	Particle.variable("solarVoltage", solarVoltage);
	Particle.variable("solarPower", solarPower);
	Particle.variable("batCurrent", batteryCurrent);
	Particle.variable("batVoltage", batteryVoltage);
	Particle.variable("batPower", batteryPower);
	Particle.variable("totalPower", totalPower);
	Particle.variable("uptime", uptime);
	Particle.variable("rawtemp", rawtemp);
	Particle.variable("rawhumidity", rawhumidity);
	Particle.variable("pumpRunning", pumpRunning);
	Particle.variable("pumpRunTime", pumpRunTime);
	Particle.variable("pumpOffTime", pumpOffTime);
	Particle.function("pumpOn", cloudPumpOn);
	Particle.function("pumpOff", cloudPumpOff);
	Particle.function("publish", cloudPublish);
	for (i=0;i<10;i++) {
		readVoltages();
		delay(1000);
	}
	evaluatePumpState();
	pumpStateTimer.start();
	publishTimer.start();
	pumpOn();
	doPublish = true;
}

#define EWMA_LEVEL              96
#define EWMA_DIV                128
double ewma_add(double old_val, double new_val)
{
	return (new_val * (EWMA_DIV - EWMA_LEVEL) + old_val * EWMA_LEVEL) / EWMA_DIV;
}

void readVoltages() {
	float shuntvoltage, shuntvoltage_b, busvoltage, busvoltage_b, current, current_b;
;
	// Current and voltage probe for the first ina219 sensor connected to the Solar Pannel
	shuntvoltage = ina219.getShuntVoltage_mV();
	busvoltage = ina219.getBusVoltage_V();
	current = ina219.getCurrent_mA();
        solarVoltage = ewma_add(solarVoltage, busvoltage + (shuntvoltage / 1000));
	solarCurrent = ewma_add(solarCurrent, current);
	solarPower = ewma_add(solarPower, busvoltage * (current / 1000));
//	solarPower = ewma_add(solarPower, ina219.getPower_mW());
	// Current and voltage probe for the second ina219 sensor connected to the battery
	shuntvoltage_b = ina219_b.getShuntVoltage_mV();
	busvoltage_b = ina219_b.getBusVoltage_V();
	current_b = ina219_b.getCurrent_mA();
        batteryVoltage = ewma_add(batteryVoltage, busvoltage_b + (shuntvoltage_b / 1000));
	batteryCurrent = ewma_add(batteryCurrent, current_b);
	batteryPower = ewma_add(batteryPower, busvoltage_b * (current_b / 1000));
	rawtemp = ewma_add(rawtemp, am2320.readTemperature());
	rawhumidity = ewma_add(rawhumidity, am2320.readHumidity());
	totalPower = solarPower + batteryPower;
}

void publishSensors() {

        // Print the values into the spark values
        Serial.print("A Current: ");Serial.println(solarCurrent);
        Serial.print("A Voltage: ");Serial.println(solarVoltage);
        Serial.print("A Power:   ");Serial.println(solarPower);
        Serial.print("B Current: ");Serial.println(batteryCurrent);
        Serial.print("B Voltage: ");Serial.println(batteryVoltage);
        Serial.print("B Power:   ");Serial.println(batteryPower);
        Serial.print("Total Pwr: ");Serial.println(totalPower);
        Serial.print("Uptime:    ");Serial.println(millis() / 1000);
	Serial.print("Temp:      "); Serial.println(rawtemp);
	Serial.print("Humidity:  "); Serial.println(rawhumidity);
	snprintf(data, 254, "{ \"tags\" : {\"id\": \"%s\"},\"values\": {"
		"\"Is\": %.2f,\"Vs\": %.2f,\"Ps\": %.2f,"
		"\"Ib\": %.2f,\"Vb\": %.2f,\"Pb\": %.2f,"
		"\"Pt\": %.2f,\"up\": %d,\"T\": %.2f,"
		"\"H\": %.2f,\"POn\": %s,\"PRun\": %d,\"POff\": %d"
		"}}",
		SENSOR_ID, solarCurrent, solarVoltage, solarPower, batteryCurrent, batteryVoltage, batteryPower,
		solarPower + batteryPower, uptime, rawtemp, rawhumidity, pumpRunning ? "true" : "false",
		pumpRunTime,pumpOffTime);
	data[254] = '\0';
	Serial.println(data);
	if (!(Particle.publish("sensors", data, PRIVATE)))
		publishTimer.changePeriod(1000);

}

void schedulePublish() {
	doPublish = true;
	publishTimer.changePeriod(5*60*1000);
}

void evaluatePumpState() {
	readVoltages();
	/* No solar */
	if (solarPower < 1.0) {
		pumpRunTime = 5;
		pumpOffTime = 25;
	/* Some sun (5W) */
	} else if (solarPower < 5.0) {
		pumpRunTime = 10;
		pumpOffTime = 15;
	/* Break even sun (10W) */
	} else if (solarPower < 15.0) {
		pumpRunTime = 15;
		pumpOffTime = 10;
	/* Some sun (40W) */
	} else if (solarPower < 40.0) {
		pumpRunTime = 30;
		pumpOffTime = 5;
	/* Full sun (>20W) */
	} else {
		pumpRunTime = 59;
		pumpOffTime = 1;
	}
}

void pumpOn() {
	pumpRunning = true;
	digitalWrite(pump, HIGH);
	pumpOffTimer.changePeriod(pumpRunTime * 60 * 1000);
}

void pumpOff() {
	pumpRunning = false;
	digitalWrite(pump, LOW);
	pumpOnTimer.changePeriod(pumpOffTime * 60 * 1000);
}

int cloudPumpOn(String extra) {
	pumpOnTimer.stop();
	pumpOn();
	return 0;
}

int cloudPumpOff(String extra) {
	pumpOffTimer.stop();
	pumpOff();
	return 0;
}

int cloudPublish(String extra) {
	schedulePublish();
	return 0;
}

void loop() {
	if (Particle.connected() == false) {
		Particle.connect();
	}
	if (doPublish) {
		doPublish = false;
		publishSensors();
	}
	uptime = millis()/1000;
}
