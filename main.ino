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
double rawtemp = 0.0, rawhumidity = 0.0, shuntvoltage = 0.0, busvoltage = 0.0, solarCurrent = 0.0;
double shuntvoltage_b = 0.0, busvoltage_b = 0.0, batteryCurrent = 0.0;
double solarPower = 0.0, batteryPower = 0.0, totalPower;
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
	Particle.variable("totalPower", batteryPower);
	Particle.variable("totalPower", totalPower);
	Particle.variable("uptime", uptime);
	Particle.variable("rawtemp", rawtemp);
	Particle.variable("rawhumidity", rawhumidity);
	Particle.variable("pumpRunning", pumpRunning);
	Particle.variable("pumpRunTime", pumpRunTime);
	Particle.variable("pumpOffTime", pumpOffTime);
	Particle.function("pumpOn", cloudPumpOn);
	Particle.function("pumpOff", cloudPumpOff);
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
	// Current and voltage probe for the first ina219 sensor connected to the Solar Pannel
	shuntvoltage = ewma_add(shuntvoltage, ina219.getShuntVoltage_mV());
	busvoltage = ewma_add(busvoltage, ina219.getBusVoltage_V());
	solarCurrent = ewma_add(solarCurrent, ina219.getCurrent_mA());
	solarPower = ewma_add(solarPower, ina219.getPower_mW());
	// Current and voltage probe for the second ina219 sensor connected to the battery
	shuntvoltage_b = ewma_add(shuntvoltage_b, ina219_b.getShuntVoltage_mV());
	busvoltage_b = ewma_add(busvoltage_b, ina219_b.getBusVoltage_V());
	batteryCurrent = ewma_add(batteryCurrent, ina219_b.getCurrent_mA());
	batteryPower = ewma_add(batteryPower, ina219_b.getPower_mW());
	rawtemp = ewma_add(rawtemp, am2320.readTemperature());
	rawhumidity = ewma_add(rawhumidity, am2320.readHumidity());
        solarVoltage = busvoltage + (shuntvoltage / 1000);
        batteryVoltage = busvoltage_b + (shuntvoltage_b / 1000);
}

void publishSensors() {

        // Print the values into the spark values
        Serial.print("A Current: ");Serial.println(solarCurrent);
        Serial.print("A Voltage: ");Serial.println(solarVoltage);
        Serial.print("A Power:   ");Serial.println(solarPower);
        Serial.print("B Current: ");Serial.println(batteryCurrent);
        Serial.print("B Voltage: ");Serial.println(batteryVoltage);
        Serial.print("B Power:   ");Serial.println(batteryPower);
        Serial.print("Total Pwr: ");Serial.println(solarPower + batteryPower);
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
	Particle.publish("sensors", data, PRIVATE);

}

void schedulePublish() {
	doPublish = true;
}

void evaluatePumpState() {
	readVoltages();
	/* No solar */
	if (solarPower < 1000) {
		pumpRunTime = 5*60*1000;
		pumpOffTime = 25*60*1000;
	/* Some sun (5W) */
	} else if (solarPower < 5000) {
		pumpRunTime = 10*60*1000;
		pumpOffTime = 15*60*1000;
	/* Break even sun (10W) */
	} else if (solarPower < 10000) {
		pumpRunTime = 15*60*1000;
		pumpOffTime = 10*60*1000;
	/* Some sun (20W) */
	} else if (solarPower < 20000) {
		pumpRunTime = 30*60*1000;
		pumpOffTime = 5*60*1000;
	/* Full sun (>20W) */
	} else {
		pumpRunTime = 59*60*1000;
		pumpOffTime = 1*60*1000;
	}
}

void pumpOn() {
	pumpRunning = true;
	digitalWrite(pump, HIGH);
	pumpOffTimer.changePeriod(pumpRunTime);
}

void pumpOff() {
	pumpRunning = false;
	digitalWrite(pump, LOW);
	pumpOnTimer.changePeriod(pumpOffTime);
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
