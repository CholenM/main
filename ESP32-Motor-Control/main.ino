
#include <WiFi.h>
#include <WebServer.h>

// ================= WIFI SETTINGS =================

const char* ssid = "scara";
const char* password = "12345678";

WebServer server(80);

// ================= Z-AXIS PINS =================

const int stepPinZ = 2;
const int dirPinZ = 15;
const int limitPinZ = 27;

// ================= XY-AXIS PINS =================

const int stepPinXY = 16;
const int dirPinXY = 17;
const int limitPinXY = 26;

// ================= EL-AXIS PINS =================

const int stepPinEL = 21;
const int dirPinEL = 22;
const int limitPinEL = 25;

// ================= END EFFECTOR PIN =================

const int drillPin = 23;  // Active-HIGH output to relay/MOSFET driver

// ================= MOTOR SETTINGS =================

const int stepDelayZ = 800;
const int stepDelayXY = 800;
const int stepDelayEL = 800;

// ================= MOTOR DIRECTIONS =================

// Reversed direction assignments

const int DIR_Z_UP = LOW;
const int DIR_Z_DOWN = HIGH;

const int DIR_XY_LEFT = LOW;
const int DIR_XY_RIGHT = HIGH;

const int DIR_EL_LEFT = LOW;
const int DIR_EL_RIGHT = HIGH;

// Limit switch protected directions
// Assumes Z limit is at UP,
// XY limit is at LEFT,
// EL limit is at LEFT.

const int LIMIT_DIR_Z = DIR_Z_DOWN;
const int LIMIT_DIR_XY = DIR_XY_LEFT;
const int LIMIT_DIR_EL = DIR_EL_RIGHT;

// ================= MOTOR STATE =================

// 0 = STOP
// 1 = Z MOTOR
// 2 = XY MOTOR
// 3 = EL MOTOR

volatile int motorState = 0;
bool drillState = false;  // false = OFF, true = ON

TaskHandle_t MotorTask;


// ================= WEBPAGE =================

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>

<head>

<meta name="viewport" content="width=device-width, initial-scale=1">

<title>SCARA MASTER</title>

<style>

:root {
    --orange: #f47b20;
    --orange-dark: #d9630e;
    --black: #171717;
    --white: #ffffff;
    --background: #f5f5f5;
    --gray: #777777;
    --border: #e7e7e7;
}

* {
    box-sizing: border-box;
}

body {
    background: var(--background);
    color: var(--black);
    font-family: Arial, Helvetica, sans-serif;
    margin: 0;
    padding: 0;
}

header {
    background: var(--white);
    padding: 22px 18px;
    border-bottom: 1px solid var(--border);
}

.header-content {
    max-width: 850px;
    margin: auto;
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 12px;
}

.logo {
    font-size: 23px;
    font-weight: 900;
    letter-spacing: 1px;
}

.logo span {
    color: var(--orange);
}

.subtitle {
    font-size: 11px;
    color: var(--gray);
    margin-top: 5px;
    letter-spacing: 1.5px;
}

.connection {
    display: flex;
    align-items: center;
    gap: 7px;
    font-size: 11px;
    font-weight: bold;
    color: var(--gray);
}

.dot {
    height: 9px;
    width: 9px;
    background: #999;
    border-radius: 50%;
}

.dot.online {
    background: #22a06b;
}

main {
    max-width: 850px;
    margin: 25px auto;
    padding: 0 15px 35px;
}

.section-heading {
    font-size: 12px;
    letter-spacing: 2px;
    font-weight: bold;
    color: var(--gray);
    margin: 25px 0 12px;
}

.axis-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 14px;
}

.card {
    background: var(--white);
    border: 1px solid var(--border);
    border-radius: 15px;
    padding: 18px;
    box-shadow: 0 3px 12px rgba(0,0,0,0.035);
}

.card-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 17px;
}

.axis-name {
    font-size: 17px;
    font-weight: 800;
}

.axis-label {
    font-size: 10px;
    color: var(--gray);
    letter-spacing: 1px;
    margin-top: 4px;
}

.axis-number {
    background: #fff0e5;
    color: var(--orange-dark);
    font-size: 12px;
    font-weight: bold;
    padding: 8px 10px;
    border-radius: 8px;
}

.control-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 9px;
}

button {
    border: none;
    border-radius: 10px;
    height: 75px;
    background: var(--orange);
    color: white;
    font-size: 13px;
    font-weight: 800;
    cursor: pointer;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    gap: 5px;
    transition: background 0.15s, transform 0.1s;
    -webkit-tap-highlight-color: transparent;
    touch-action: manipulation;
}

button:hover {
    background: var(--orange-dark);
}

button:active {
    transform: scale(0.96);
}

.arrow {
    font-size: 24px;
    line-height: 24px;
}

.stop-container {
    margin-top: 18px;
}

.stop {
    width: 100%;
    height: 62px;
    background: var(--black);
    color: white;
    font-size: 15px;
    letter-spacing: 1px;
    border-radius: 11px;
    flex-direction: row;
    gap: 10px;
}

.stop:hover {
    background: #333333;
}

.stop-icon {
    font-size: 20px;
}

.drill-panel {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 14px;
}

.drill-button {
    width: 160px;
    height: 58px;
    flex-shrink: 0;
    background: var(--orange);
}

.drill-button.on {
    background: var(--black);
}

.drill-button.on:hover {
    background: #333333;
}

.limit-section {
    background: var(--white);
    border: 1px solid var(--border);
    border-radius: 15px;
    padding: 18px;
}

.limit-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 12px 0;
    border-bottom: 1px solid var(--border);
}

.limit-row:last-child {
    border-bottom: none;
}

.limit-name {
    font-size: 13px;
    font-weight: bold;
}

.limit-state {
    padding: 6px 10px;
    border-radius: 6px;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 0.5px;
}

.clear {
    background: #e8f6ed;
    color: #21844d;
}

.triggered {
    background: #fff0e5;
    color: #d9630e;
}

.message {
    text-align: center;
    margin-top: 18px;
    font-size: 12px;
    color: var(--gray);
    min-height: 18px;
}

footer {
    text-align: center;
    color: #999;
    font-size: 10px;
    letter-spacing: 1px;
    padding: 20px;
}

@media(max-width: 650px) {

    .axis-grid {
        grid-template-columns: 1fr;
    }

    main {
        margin-top: 10px;
    }

    .card {
        padding: 20px;
    }

    button {
        height: 68px;
    }

    .drill-panel {
        align-items: stretch;
        flex-direction: column;
    }

    .drill-button {
        width: 100%;
    }

    .logo {
        font-size: 20px;
    }

}

</style>

</head>

<body>

<header>

    <div class="header-content">

        <div>
            <div class="logo">SCARA <span>MASTER</span></div>
            <div class="subtitle">ROBOTIC ARM CONTROL SYSTEM</div>
        </div>

        <div class="connection">
            <span class="dot" id="connectionDot"></span>
            <span id="connectionText">CONNECTING</span>
        </div>

    </div>

</header>


<main>

    <div class="section-heading">MOTOR CONTROL</div>

    <div class="axis-grid">

        <!-- Z AXIS -->

        <div class="card">

            <div class="card-header">

                <div>
                    <div class="axis-name">Z-AXIS</div>
                    <div class="axis-label">VERTICAL MOVEMENT</div>
                </div>

                <div class="axis-number">01</div>

            </div>

            <div class="control-grid">

                <button onclick="go('Z_UP')">
                    <span>UP</span>
                </button>

                <button onclick="go('Z_DOWN')">
                    <span>DOWN</span>
                </button>

            </div>

        </div>


        <!-- XY AXIS -->

        <div class="card">

            <div class="card-header">

                <div>
                    <div class="axis-name">XY-AXIS</div>
                    <div class="axis-label">SHOULDER MOVEMENT</div>
                </div>

                <div class="axis-number">02</div>

            </div>

            <div class="control-grid">

                <button onclick="go('XY_LEFT')">
                    <span>LEFT</span>
                </button>

                <button onclick="go('XY_RIGHT')">
                    <span>RIGHT</span>
                </button>

            </div>

        </div>


        <!-- EL AXIS -->

        <div class="card">

            <div class="card-header">

                <div>
                    <div class="axis-name">EL-AXIS</div>
                    <div class="axis-label">ELBOW MOVEMENT</div>
                </div>

                <div class="axis-number">03</div>

            </div>

            <div class="control-grid">

                <button onclick="go('EL_LEFT')">
                    <span>LEFT</span>
                </button>

                <button onclick="go('EL_RIGHT')">
                    <span>RIGHT</span>
                </button>

            </div>

        </div>

    </div>


    <!-- STOP ALL -->

    <div class="stop-container">

        <button class="stop" onclick="go('STOP')">
            STOP ALL MOTORS
        </button>

    </div>


    <!-- END EFFECTOR DRILL CONTROL -->

    <div class="section-heading">END EFFECTOR</div>

    <div class="limit-section drill-panel">
        <div>
            <div class="limit-name">DRILL MOTOR</div>
            <div class="axis-label">END EFFECTOR CONTROL</div>
        </div>

        <button class="drill-button" id="drillButton" onclick="toggleDrill()" aria-pressed="false">
            DRILL OFF
        </button>
    </div>


    <!-- LIMIT SWITCH STATUS -->

    <div class="section-heading">LIMIT SWITCH MONITOR</div>

    <div class="limit-section">

        <div class="limit-row">
            <span class="limit-name">Z-AXIS LIMIT</span>
            <span class="limit-state clear" id="zStatus">CLEAR</span>
        </div>

        <div class="limit-row">
            <span class="limit-name">XY-AXIS LIMIT</span>
            <span class="limit-state clear" id="xyStatus">CLEAR</span>
        </div>

        <div class="limit-row">
            <span class="limit-name">EL-AXIS LIMIT</span>
            <span class="limit-state clear" id="elStatus">CLEAR</span>
        </div>

    </div>


    <div class="message" id="message">
        SCARA controller initializing...
    </div>

</main>


<footer>
    SCARA MASTER &bull; ESP32 CONTROL INTERFACE
</footer>


<script>

// SEND MOTOR COMMAND

async function go(command) {

    const message = document.getElementById('message');

    try {

        const response = await fetch(
            '/cmd?m=' + encodeURIComponent(command)
        );

        if (!response.ok) {
            throw new Error('Command failed');
        }

        message.textContent = 'COMMAND: ' + command.replace('_', ' ');

        await refreshStatus();

    }

    catch (error) {

        message.textContent = 'Unable to communicate with controller.';

    }

}


// UPDATE DRILL BUTTON STATE

let drillOn = false;

function updateDrill(value) {

    drillOn = (value === 1);

    const button = document.getElementById('drillButton');
    button.textContent = drillOn ? 'DRILL ON' : 'DRILL OFF';
    button.className = drillOn ? 'drill-button on' : 'drill-button';
    button.setAttribute('aria-pressed', drillOn ? 'true' : 'false');

}

async function toggleDrill() {
    await go(drillOn ? 'DRILL_OFF' : 'DRILL_ON');
}


// UPDATE LIMIT SWITCH INDICATORS

function updateLimit(id, value) {

    const element = document.getElementById(id);

    if (value === 0) {

        element.textContent = 'TRIGGERED';
        element.className = 'limit-state triggered';

    }

    else {

        element.textContent = 'CLEAR';
        element.className = 'limit-state clear';

    }

}


// REFRESH CONTROLLER STATUS

async function refreshStatus() {

    const dot = document.getElementById('connectionDot');
    const text = document.getElementById('connectionText');

    try {

        const response = await fetch(
            '/status',
            { cache: 'no-store' }
        );

        if (!response.ok) {
            throw new Error('Status unavailable');
        }

        const data = await response.json();

        dot.className = 'dot online';
        text.textContent = 'ONLINE';

        updateLimit('zStatus', data.zLimit);
        updateLimit('xyStatus', data.xyLimit);
        updateLimit('elStatus', data.elLimit);
        updateDrill(data.drillState);

    }

    catch (error) {

        dot.className = 'dot';
        text.textContent = 'OFFLINE';

    }

}


// AUTOMATIC STATUS REFRESH

refreshStatus();

setInterval(refreshStatus, 600);

</script>

</body>

</html>
)rawliteral";


// ================= COMMAND HANDLER =================

void handleCmd() {

    if (server.hasArg("m")) {

        String cmd = server.arg("m");

        // END EFFECTOR DRILL

        if (cmd == "DRILL_ON") {

            digitalWrite(drillPin, HIGH);
            drillState = true;

        }

        else if (cmd == "DRILL_OFF") {

            digitalWrite(drillPin, LOW);
            drillState = false;

        }


        // Z-AXIS

        if (cmd == "Z_UP") {

            digitalWrite(dirPinZ, DIR_Z_UP);
            motorState = 1;

        }

        else if (cmd == "Z_DOWN") {

            digitalWrite(dirPinZ, DIR_Z_DOWN);
            motorState = 1;

        }


        // XY-AXIS

        else if (cmd == "XY_LEFT") {

            digitalWrite(dirPinXY, DIR_XY_LEFT);
            motorState = 2;

        }

        else if (cmd == "XY_RIGHT") {

            digitalWrite(dirPinXY, DIR_XY_RIGHT);
            motorState = 2;

        }


        // EL-AXIS

        else if (cmd == "EL_LEFT") {

            digitalWrite(dirPinEL, DIR_EL_LEFT);
            motorState = 3;

        }

        else if (cmd == "EL_RIGHT") {

            digitalWrite(dirPinEL, DIR_EL_RIGHT);
            motorState = 3;

        }


        // STOP ALL

        else if (cmd == "STOP") {

            motorState = 0;
            digitalWrite(drillPin, LOW);
            drillState = false;

        }

    }

    server.send(200, "text/plain", "OK");

}


// ================= STATUS HANDLER =================

void handleStatus() {

    char json[160];

    snprintf(
        json,
        sizeof(json),
        "{\"motorState\":%d,\"zLimit\":%d,\"xyLimit\":%d,\"elLimit\":%d,\"drillState\":%d}",
        (int)motorState,
        digitalRead(limitPinZ),
        digitalRead(limitPinXY),
        digitalRead(limitPinEL),
        drillState ? 1 : 0
    );

    server.send(200, "application/json", json);

}


// ================= MOTOR STEP FUNCTION =================

void stepMotor(
    int stepPin,
    int dirPin,
    int limitPin,
    int limitDirection,
    int stepDelay,
    int activeState
) {

    // Check active motor

    if (motorState != activeState) {

        digitalWrite(stepPin, LOW);
        return;

    }


    // Check limit switch
    // LOW = pressed
    // HIGH = released
    //
    // Stop only when travelling toward the switch.

    if (
        digitalRead(limitPin) == LOW &&
        digitalRead(dirPin) == limitDirection
    ) {

        digitalWrite(stepPin, LOW);

        if (motorState == activeState) {

            motorState = 0;

        }

        return;

    }


    // Recheck motor state

    if (motorState != activeState) {

        digitalWrite(stepPin, LOW);
        return;

    }


    // Generate STEP pulse

    digitalWrite(stepPin, HIGH);

    delayMicroseconds(stepDelay);

    digitalWrite(stepPin, LOW);

    delayMicroseconds(stepDelay);

}


// ================= MOTOR TASK =================

void motorLoop(void *pvParameters) {

    for (;;) {

        int currentState = motorState;


        // Z-AXIS

        if (currentState == 1) {

            stepMotor(
                stepPinZ,
                dirPinZ,
                limitPinZ,
                LIMIT_DIR_Z,
                stepDelayZ,
                1
            );

        }


        // XY-AXIS

        else if (currentState == 2) {

            stepMotor(
                stepPinXY,
                dirPinXY,
                limitPinXY,
                LIMIT_DIR_XY,
                stepDelayXY,
                2
            );

        }


        // EL-AXIS

        else if (currentState == 3) {

            stepMotor(
                stepPinEL,
                dirPinEL,
                limitPinEL,
                LIMIT_DIR_EL,
                stepDelayEL,
                3
            );

        }


        // STOPPED

        else {

            digitalWrite(stepPinZ, LOW);
            digitalWrite(stepPinXY, LOW);
            digitalWrite(stepPinEL, LOW);

            delay(1);

        }

    }

}


// ================= SETUP =================

void setup() {

    Serial.begin(115200);


    // CONFIGURE MOTOR PINS

    int pList[] = {

        stepPinZ, dirPinZ,
        stepPinXY, dirPinXY,
        stepPinEL, dirPinEL

    };

    for (int p : pList) {

        pinMode(p, OUTPUT);

    }


    // INITIALIZE STEP PINS

    digitalWrite(stepPinZ, LOW);
    digitalWrite(stepPinXY, LOW);
    digitalWrite(stepPinEL, LOW);


    // INITIALIZE DIRECTION PINS

    digitalWrite(dirPinZ, DIR_Z_DOWN);
    digitalWrite(dirPinXY, DIR_XY_RIGHT);
    digitalWrite(dirPinEL, DIR_EL_RIGHT);


    // CONFIGURE DRILL OUTPUT (DEFAULT OFF)

    pinMode(drillPin, OUTPUT);
    digitalWrite(drillPin, LOW);
    drillState = false;


    // CONFIGURE LIMIT SWITCHES

    pinMode(limitPinZ, INPUT_PULLUP);
    pinMode(limitPinXY, INPUT_PULLUP);
    pinMode(limitPinEL, INPUT_PULLUP);


    // PRINT INITIAL LIMIT SWITCH STATUS

    Serial.println();
    Serial.println("LIMIT SWITCH STATUS");

    Serial.print("Z: ");
    Serial.println(digitalRead(limitPinZ));

    Serial.print("XY: ");
    Serial.println(digitalRead(limitPinXY));

    Serial.print("EL: ");
    Serial.println(digitalRead(limitPinEL));


    // CREATE MOTOR TASK

    BaseType_t taskResult = xTaskCreatePinnedToCore(
        motorLoop,
        "MotorTask",
        10000,
        NULL,
        1,
        &MotorTask,
        0
    );

    if (taskResult != pdPASS) {

        Serial.println("ERROR: Motor task creation failed!");

        while (true) {
            delay(1000);
        }

    }


    // START WIFI ACCESS POINT

    WiFi.softAP(ssid, password, 6);

    Serial.println();
    Serial.println("SCARA MASTER STARTED");

    Serial.print("WiFi SSID: ");
    Serial.println(ssid);

    Serial.print("IP ADDRESS: ");
    Serial.println(WiFi.softAPIP());


    // WEB SERVER ROUTES

    server.on("/", []() {

        server.send(200, "text/html", webpage);

    });

    server.on("/cmd", handleCmd);

    server.on("/status", handleStatus);


    server.begin();

    Serial.println("WEB SERVER STARTED");

}


// ================= MAIN LOOP =================

void loop() {

    server.handleClient();

    delay(2);

}
