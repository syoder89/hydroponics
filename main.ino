#include "Adafruit_INA219.h"
#include "Adafruit_AM2320.h"
#include "HttpClient.h"
#include "math.h"

#define SENSOR_ID "ScottHydro"
#define INFLUXDB_HOST "205.159.243.132"
#define INFLUXDB_PORT 8086
#define INFLUXDB_DB "hydroponics"
#define SENSOR_NAME "ScottHydro"
Adafruit_INA219 ina219;
Adafruit_INA219 ina219_b(0x41);
Adafruit_AM2320 am2320 = Adafruit_AM2320();
HttpClient http;

SYSTEM_MODE(SEMI_AUTOMATIC);

#define SENSOR_UPDATE_INTERVAL 3
#define INFLUX_UPDATE_INTERVAL 30
#define FLOWRATE_UPDATE_INTERVAL INFLUX_UPDATE_INTERVAL
#define HTTP_TIMEOUT 5000

// 36 AH Deka Solar 12V Gel Deep Cycle Battery
// My dual batteries are pretty bad also
#define BATTERY_CAPACITY 17350
// My battery isn't doing so good these days
//#define BATTERY_CAPACITY 10580
#define DT SENSOR_UPDATE_INTERVAL / 3600

int pumpPin = D8, flowInPin = D6, flowOutPin = D5;
Timer sensorsTimer(SENSOR_UPDATE_INTERVAL*1000, readSensors);
Timer pumpStateTimer(60*1000, evaluatePumpState);
Timer pumpOffTimer(5*60*1000, pumpOff, true);
Timer pumpOnTimer(5*60*1000, pumpOn, true);
//Timer publishTimer(5*60*1000, schedulePublish);
Timer influxTimer(INFLUX_UPDATE_INTERVAL*1000, scheduleInflux);
int pumpRunTime;
int pumpOffTime;
boolean pumpRunning = false, pumpAuto = true;
boolean didFullCharge = false;
double rawtemp = 0.0, rawhumidity = 0.0, solarCurrent = 0.0;
double batteryCurrent = 0.0, stateOfCharge = 0.0, accumulatedAH = 0.0;
double solarPower = 0.0, batteryPower = 0.0, totalPower = 0.0;
double solarVoltage = 0.0, batteryVoltage = 0.0;
double wSolarPower = 0.0, wBatteryVoltage = 0.0;
double shuntvoltage, shuntvoltage_b, busvoltage, busvoltage_b, current, current_b;
double batteryCapacity = BATTERY_CAPACITY;
int uptime;
boolean doPublish = false, doInflux = false;
double flowRateIn, flowRateOut, flowIn, flowOut;
int pulsesIn = 0, pulsesOut = 0, reconnects = 0, influxFails = 0;
unsigned long lastPulses;

void setup() {
	Serial.begin(115200);
	pinMode(pumpPin, OUTPUT);
	pinMode(flowInPin, INPUT_PULLUP);
	pinMode(flowOutPin, INPUT_PULLUP);
	lastPulses = millis();
	attachInterrupt(flowInPin, handleFlowRateIn, FALLING);
	attachInterrupt(flowOutPin, handleFlowRateOut, FALLING);
	ina219.begin();
	ina219_b.begin();
	am2320.begin();
//	Particle.variable("wSolarPower", wSolarPower);
//	Particle.variable("wBatVoltage", wBatteryVoltage);
//	Particle.variable("solarCurrent", solarCurrent);
//	Particle.variable("solarVoltage", solarVoltage);
//	Particle.variable("solarPower", solarPower);
//	Particle.variable("batCurrent", batteryCurrent);
//	Particle.variable("batVoltage", batteryVoltage);
//	Particle.variable("batPower", batteryPower);
//	Particle.variable("totalPower", totalPower);
//	Particle.variable("uptime", uptime);
//	Particle.variable("rawtemp", rawtemp);
//	Particle.variable("rawhumidity", rawhumidity);
	Particle.variable("pumpRunning", pumpRunning);
	Particle.variable("flowRateIn", flowRateIn);
	Particle.variable("flowRateOut", flowRateOut);
	Particle.variable("pulsesIn", pulsesIn);
	Particle.variable("pulsesOut", pulsesOut);
//	Particle.variable("accumulatedAH", accumulatedAH);
//	Particle.variable("batteryCapacity", batteryCapacity);
//	Particle.variable("pumpRunTime", pumpRunTime);
//	Particle.variable("pumpOffTime", pumpOffTime);
	Particle.function("pumpAuto", cloudPumpAuto);
	Particle.function("pumpOn", cloudPumpOn);
	Particle.function("pumpOff", cloudPumpOff);
//	Particle.function("publish", cloudPublish);
	Particle.function("influx", cloudInflux);
	readSensors();
	updateSensorsInit();
	evaluatePumpState();
	pumpStateTimer.start();
//	publishTimer.start();
	influxTimer.start();
	sensorsTimer.start();
	pumpOn();
	uptime = millis()/1000;
//	doPublish = true;
	doInflux = true;
}

#define EWMA_LEVEL              96
#define EWMA_DIV                128
double ewma_add(double old_val, double new_val)
{
	return (new_val * (EWMA_DIV - EWMA_LEVEL) + old_val * EWMA_LEVEL) / EWMA_DIV;
}

void handleFlowRateIn() {
	pulsesIn++;
}

void handleFlowRateOut() {
	pulsesOut++;
}

void updateFlowRate() {
	flowRateIn = pulsesIn / (7.5 * ((millis() - lastPulses)/1000));
	flowRateOut = pulsesOut / (7.5 * ((millis() - lastPulses)/1000));
//	flowRateIn = ewma_add(flowRateIn, flowIn);
//	flowRateOut = ewma_add(flowRateOut, flowOut);
}

void resetFlowRate() {
	pulsesIn = 0;
	pulsesOut = 0;
	lastPulses = millis();
}

void initialStateOfCharge() {
	double v = batteryVoltage;

	/* Theory - compensate for solar charging linearly with a 1.6V float at the top */
	/* Charge voltage appears fairly linear up to max, panel is 45W */
//	v -= (solarPower / 45.0) * 1.6;
/*
	if (solarPower > 5)
		v -= (solarPower / 120.0) * (solarVoltage - batteryVoltage);
*/
	if (solarVoltage > 12.8)
		v -= (solarVoltage - 12.8);
	/* Theory - linear discharge, close but not quite, from 12.8V down to 11.0V */

	stateOfCharge = (v - 11.0) / 1.5 * 100;
	if (stateOfCharge > 100.0)
		stateOfCharge = 100.0;
	if (stateOfCharge < 0.0)
		stateOfCharge = 0.0;
}

void updateStateOfCharge() {
	/* Accumulated AH keeps track of battery capacity. It should be 0 at full charge and goes
	   negative as we draw power from it. */
	accumulatedAH -= (batteryCurrent * DT);
	/* Not intuitive. Current is negative when we're charging so it will be approaching 0 for full charge. */
	if (accumulatedAH > 0.0)
		accumulatedAH = 0.0;
	stateOfCharge -= (100 * ((batteryCurrent / batteryCapacity) * DT));
	if (stateOfCharge > 100.0 || (batteryVoltage > 14.0 && solarCurrent < 20.0)) {
		stateOfCharge = 100.0;
		accumulatedAH = 0.0;
		didFullCharge = true;
	}
	if (stateOfCharge < 0.0)
		stateOfCharge = 0.0;
	/* Battery is dead, charge controller shut down */
	if (batteryVoltage < 11.0) {
		stateOfCharge = 0.0;
		/* Recalibrate capacity based on Amp-Hours drawn from it. Note this only works if we hit full charge first */
		if (didFullCharge)
			batteryCapacity = -1.0 * accumulatedAH;
		didFullCharge = false;
	}
}

void updateSensorsInit() {
	solarVoltage = busvoltage + (shuntvoltage / 1000);
	solarCurrent = current;
	solarPower = busvoltage * (current / 1000);
	batteryVoltage = busvoltage_b + (shuntvoltage_b / 1000);
	batteryCurrent = current_b;
	batteryPower = busvoltage_b * (current_b / 1000);
	rawtemp = am2320.readTemperature();
	rawhumidity = am2320.readHumidity();
	totalPower = solarPower + batteryPower;
	wSolarPower = solarPower;
        wBatteryVoltage = batteryVoltage;
	flowRateIn = 0;
	flowRateOut = 0;
	initialStateOfCharge();
}

void updateSensors() {
	solarVoltage = busvoltage + (shuntvoltage / 1000);
	solarCurrent = current;
	solarPower = busvoltage * (current / 1000);
	batteryVoltage = busvoltage_b + (shuntvoltage_b / 1000);
	batteryCurrent = current_b;
	batteryPower = busvoltage_b * (current_b / 1000);
	rawtemp = am2320.readTemperature();
	rawhumidity = am2320.readHumidity();
	totalPower = solarPower + batteryPower;
}

void updateSensors_w() {
	solarVoltage = ewma_add(solarVoltage, busvoltage + (shuntvoltage / 1000));
	solarCurrent = ewma_add(solarCurrent, current);
	solarPower = ewma_add(solarPower, busvoltage * (current / 1000));
	batteryVoltage = ewma_add(batteryVoltage, busvoltage_b + (shuntvoltage_b / 1000));
	batteryCurrent = ewma_add(batteryCurrent, current_b);
	batteryPower = ewma_add(batteryPower, busvoltage_b * (current_b / 1000));
	rawtemp = ewma_add(rawtemp, am2320.readTemperature());
	rawhumidity = ewma_add(rawhumidity, am2320.readHumidity());
	totalPower = solarPower + batteryPower;
}

void readSensors() {
	// Current and voltage probe for the first ina219 sensor connected to the Solar Pannel
	shuntvoltage = ina219.getShuntVoltage_mV();
	busvoltage = ina219.getBusVoltage_V();
	current = ina219.getCurrent_mA();
	// Current and voltage probe for the second ina219 sensor connected to the battery
	shuntvoltage_b = ina219_b.getShuntVoltage_mV();
	busvoltage_b = ina219_b.getBusVoltage_V();
	current_b = ina219_b.getCurrent_mA();
	updateSensors();
	updateStateOfCharge();
}

// Headers currently need to be set at init, useful for API keys etc.
http_header_t headers[] = {
	{ "Connection", "close" },
	{ "Accept" , "application/json" },
	{ NULL, NULL } // NOTE: Always terminate headers will NULL
};

bool sendInflux(char *payload) {
	http_request_t     request;
	http_response_t    response;

	request.hostname = INFLUXDB_HOST;
	request.port     = INFLUXDB_PORT;
	request.path     = "/write?db=" + String(INFLUXDB_DB);
	request.body     = payload;
	request.timeout  = HTTP_TIMEOUT;

	http.post(request, response, headers);

	if (response.status >= 200 && response.status < 300) {
		return true;
	} else {
		return false;
	}
}

#define INFLUX_MAXLEN 1024
void publishInflux() {
	static char payload[INFLUX_MAXLEN];

	updateFlowRate();

	snprintf(payload, INFLUX_MAXLEN, "solar_current,sensor=%s value=%.2f", SENSOR_NAME, solarCurrent);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nsolar_voltage,sensor=%s value=%.2f", SENSOR_NAME, solarVoltage);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nsolar_power,sensor=%s value=%.2f", SENSOR_NAME, solarPower);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nbattery_current,sensor=%s value=%.2f", SENSOR_NAME, batteryCurrent);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nbattery_voltage,sensor=%s value=%.2f", SENSOR_NAME, batteryVoltage);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nbattery_power,sensor=%s value=%.2f", SENSOR_NAME, batteryPower);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nused_power,sensor=%s value=%.2f", SENSOR_NAME, totalPower);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nrun_time,sensor=%s value=%d", SENSOR_NAME, uptime);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\ntemperature,sensor=%s value=%.2f", SENSOR_NAME, rawtemp);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nhumidity,sensor=%s value=%.2f", SENSOR_NAME, rawhumidity);
        if (!(sendInflux(payload))) {
		influxFails++;
		influxTimer.changePeriod(1000);
		return;
	}

	snprintf(payload, INFLUX_MAXLEN - strlen(payload), "pump_running,sensor=%s value=%d", SENSOR_NAME, pumpRunning ? 1 : 0);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\npump_runtime,sensor=%s value=%d", SENSOR_NAME, pumpRunTime);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\npump_offtime,sensor=%s value=%d", SENSOR_NAME, pumpOffTime);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nflowRateIn,sensor=%s value=%.2f", SENSOR_NAME, flowRateIn);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nflowRateOut,sensor=%s value=%.2f", SENSOR_NAME, flowRateOut);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nstateOfCharge,sensor=%s value=%.2f", SENSOR_NAME, stateOfCharge);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\naccumulatedAH,sensor=%s value=%.2f", SENSOR_NAME, accumulatedAH);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nbatteryCap,sensor=%s value=%.2f", SENSOR_NAME, batteryCapacity);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\nreconnects,sensor=%s value=%d", SENSOR_NAME, reconnects);
	snprintf(payload + strlen(payload), INFLUX_MAXLEN - strlen(payload), "\ninfluxFails,sensor=%s value=%d", SENSOR_NAME, influxFails);
        if (!(sendInflux(payload))) {
		influxFails++;
		influxTimer.changePeriod(1000);
		return;
	}
	resetFlowRate();
}

void scheduleInflux() {
	doInflux = true;
	influxTimer.changePeriod(INFLUX_UPDATE_INTERVAL*1000);
}

/*
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
*/

void evaluatePumpState() {
	wSolarPower = ewma_add(wSolarPower, solarPower);
        wBatteryVoltage = ewma_add(wBatteryVoltage, batteryVoltage);

	/* No solar */
	if (wSolarPower < 5.0) {
		pumpRunTime = 5;
		pumpOffTime = 15;
	/* Some sun (20W) */
	} else if (wSolarPower < 20.0) {
		pumpRunTime = 10;
		pumpOffTime = 15;
	/* Almost break even sun (30W) */
	} else if (wSolarPower < 30.0) {
		pumpRunTime = 15;
		pumpOffTime = 15;
	/* Some sun (80W) */
	} else if (wSolarPower < 80.0) {
		pumpRunTime = 20;
		pumpOffTime = 10;
	/* Full sun (>80W) */
	} else {
		pumpRunTime = 30;
		pumpOffTime = 10;
	}
	/* If it's fully charged or super hot run more */
	if (wBatteryVoltage > 14.0 || rawtemp >= 55.0) {
		pumpRunTime = 30;
		pumpOffTime = 5;
	/* If it's still pretty hot (>113F in the box) then run more */
	} else if (rawtemp >= 45.0) {
		pumpRunTime = max(pumpRunTime, 30);
		pumpOffTime = 10;
	}
	/* Below 30% SoC conserve as much as possible. This should
	   only happen in the winter when it's cold and not sunny. */
/*
	if ((!pumpRunning && wBatteryVoltage < 11.8) ||
		(pumpRunning && wBatteryVoltage < 11.5)) {
*/
	if (stateOfCharge < 30.0) {
		pumpRunTime = 1;
		pumpOffTime = 59;
	}
}

void pumpOn() {
	pumpRunning = true;
	digitalWrite(pumpPin, HIGH);
	if (pumpAuto)
		pumpOffTimer.changePeriod(pumpRunTime * 60 * 1000);
}

void pumpOff() {
	pumpRunning = false;
	digitalWrite(pumpPin, LOW);
	if (pumpAuto)
		pumpOnTimer.changePeriod(pumpOffTime * 60 * 1000);
}

int cloudPumpAuto(String extra) {
	pumpAuto = true;
	pumpOn();
	return 0;
}

int cloudPumpOn(String extra) {
	pumpAuto = false;
	pumpOnTimer.stop();
	pumpOffTimer.stop();
	pumpOn();
	return 0;
}

int cloudPumpOff(String extra) {
	pumpAuto = false;
	pumpOnTimer.stop();
	pumpOffTimer.stop();
	pumpOff();
	return 0;
}

/*
int cloudPublish(String extra) {
	schedulePublish();
	return 0;
}
*/

int cloudInflux(String extra) {
	scheduleInflux();
	return 0;
}

void loop() {
	if (Particle.connected() == false) {
		Particle.connect();
		reconnects++;
	}
	uptime = millis()/1000;
/*
	if (doPublish) {
		doPublish = false;
		publishSensors();
	}
*/
	if (doInflux) {
		doInflux = false;
		publishInflux();
	}
}
