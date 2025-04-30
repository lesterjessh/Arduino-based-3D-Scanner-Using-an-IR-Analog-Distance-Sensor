#include <SPI.h>
#include <SD.h>
#include <math.h>

// Editable variables
int scan_amount = 5;  // Number of scans to average for each point
int z_axis_height = 20;  // in cm - Maximum height of the scanned object // arbitrary
int step_delay = 50; // in us - Delay for each step for the stepper motor
float z_layer_height = 0.5;  // in mm - Layer height for each scan
int lead_screw_rotations_per_cm = 8;  // Rotations needed for 1cm of vertical movement // 2mm pitch lead screw
int steps_per_rotation_for_motor = 1600;  // Steps per full rotation // 1/8 stepping
float distance_to_center = 15.2;  // in cm - Distance from sensor to turntable center
int empty_scan_threshold = 3;  // Number of empty scans before considering object ended 
float max_object_distance = 25.0;  // Maximum distance (in cm) to consider a valid object detection

// I/O Pins
int button = 2;
int limit_switch = 10;  // Limit switch pin
// Turntable driver pins
int dir_r = 7;
int step_r = 8;
int enable_r = 9;
// Z-axis driver pins
int dir_z = 3;
int step_z = 5;
int enable_z = 6;

// Variables
File file_values;
volatile int scan = 0;
float distance = 0;
float angle = 0;
float x = 0;
float y = 0;
float z = 0;
int z_loop = 0;
int r_loop = 0;
float measured_analog = 0;
float analog = 0;
float RADIANS = 0.0;
int steps_z_height = 0;
int empty_scan_count = 0;
bool object_found = false;

// File management variables
int fileNumber = 1;
char fileName[13];  // Array to hold filename (8.3 format: "SCANxxx.TXT")

// Button handling variables
const unsigned long DEBOUNCE_DELAY = 50;  // Debounce time in milliseconds
volatile unsigned long lastDebounceTime = 0;
volatile bool lastButtonState = HIGH;
volatile bool buttonState = HIGH;

// Error flag for sensor readings
bool sensor_error = false;

// Function to home the Z-axis
void homeZAxis() {
    Serial.println("Homing Z-axis...");
    
    digitalWrite(enable_z, LOW);
    
    bool initialButtonState = digitalRead(button);
    unsigned long lastButtonCheck = millis();
    unsigned long startTime = millis();
    
    delay(500);
    
    Serial.println("Moving up slightly...");
    digitalWrite(dir_z, LOW);
    for (int i = 0; i < 50; i++) {
        digitalWrite(step_z, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(step_z, LOW);
        delayMicroseconds(step_delay);
    }

    Serial.println("Moving down until limit switch...");
    digitalWrite(dir_z, HIGH);
    
    while(digitalRead(limit_switch) == HIGH) {
        digitalWrite(step_z, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(step_z, LOW);
        delayMicroseconds(step_delay);
    }
    
    Serial.println("Limit switch triggered!");
    
    Serial.println("Moving up to clear switch...");
    digitalWrite(dir_z, LOW);
    for(int i = 0; i < 50; i++) {
        digitalWrite(step_z, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(step_z, LOW);
        delayMicroseconds(step_delay);
    }
    
    digitalWrite(enable_z, HIGH);
    z = 0;
    Serial.println("Z-axis homed successfully!");
}

// Function to check for object presence
bool checkForObject() {
    int valid_points = 0;
    int total_points = steps_per_rotation_for_motor / 16; // Check every 16th step for efficiency
    
    for(int i = 0; i < total_points; i++) {
        sensor_error = false;
        getDistance();
        if(!sensor_error && distance < max_object_distance) {
            valid_points++;
        }
        for(int j = 0; j < 16; j++) {
            digitalWrite(enable_r, LOW);
            digitalWrite(dir_r, LOW);
            digitalWrite(step_r, HIGH);
            delayMicroseconds(step_delay);
            digitalWrite(step_r, LOW);
            delayMicroseconds(step_delay);
        }
    }
    
    digitalWrite(enable_r, HIGH); // Disable turntable after checking
    return (valid_points > (total_points / 4)); // Return true if at least 25% of points detect an object
}

// Function to get the next available filename
void getNextFileName() {
    while (true) {
        sprintf(fileName, "SCAN%03d.TXT", fileNumber);
        if (!SD.exists(fileName)) {
            Serial.print("Creating new file: ");
            Serial.println(fileName);
            break;
        }
        fileNumber++;
        if (fileNumber > 999) {
            fileNumber = 1;
        }
    }
}

// Function to write the file header
void writeFileHeader() {
    file_values = SD.open(fileName, FILE_WRITE);
    if (file_values) {
        file_values.println("X,Y,Z");
        file_values.print("# Scan parameters: ");
        file_values.print("Height: "); file_values.print(z_axis_height);
        file_values.print("cm, Layer height: "); file_values.print(z_layer_height);
        file_values.println("cm");
        file_values.println("# Data points below:");
        file_values.close();
    }
    else {
        Serial.println("Error opening file!");
    }
}

void setup() {
    Serial.begin(9600);
    
    pinMode(button, INPUT_PULLUP);
    pinMode(A0, INPUT);
    pinMode(limit_switch, INPUT_PULLUP);  // Use internal pull-up resistor
    
    analogReference(INTERNAL);
    
    if (!SD.begin(4)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    
    pinMode(dir_z, OUTPUT);
    pinMode(step_z, OUTPUT);
    pinMode(enable_z, OUTPUT);
    pinMode(dir_r, OUTPUT);
    pinMode(step_r, OUTPUT);
    pinMode(enable_r, OUTPUT);
    digitalWrite(enable_z, HIGH);
    digitalWrite(enable_r, HIGH);

    RADIANS = (3.141592 / 180.0) * (360.0 / steps_per_rotation_for_motor);
    steps_z_height = (z_layer_height * steps_per_rotation_for_motor * lead_screw_rotations_per_cm) / 10;

    if (distance_to_center < 4.0 || distance_to_center > 30.0) {
        Serial.println("Error: distance_to_center must be between 4 and 30 cm");
        while(1);
    }

    Serial.println("Scanner initialized! Waiting for button press to start scanning.");
}

void loop() {
    bool reading = digitalRead(button);
    
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
            buttonState = reading;
            
            if (buttonState == LOW) { // Button pressed (assuming active LOW)
                if (scan == 1) {
                    // Stop the scan
                    scan = 0;
                    digitalWrite(enable_z, HIGH);
                    digitalWrite(enable_r, HIGH);
                    Serial.println("Scan stopped.");
                    homeZAxis(); // Home when scan is stopped
                } else {
                    // Start a new scan
                    Serial.println("Button pressed. Starting scan...");
                    scan = 1;
                    z = 0;
                    angle = 0;
                    z_loop = 0;
                    empty_scan_count = 0;
                    object_found = false;
                    
                    // Prepare for new scan
                    getNextFileName();
                    writeFileHeader();
                    
                    // Home Z-axis before starting the scan
                    homeZAxis();
                }
            }
        }
    }
    
    lastButtonState = reading;

    if(scan == 1) {
        if(z < z_axis_height) {
            // Check for object presence at the start of each layer
            if(!object_found) {
                object_found = checkForObject();
                if(!object_found) {
                    empty_scan_count++;
                    Serial.print("Empty scan count: ");
                    Serial.println(empty_scan_count);
                    
                    if(empty_scan_count >= empty_scan_threshold) {
                        Serial.println("Object no longer detected. Stopping scan.");
                        scan = 0;
                        digitalWrite(enable_z, HIGH);
                        digitalWrite(enable_r, HIGH);
                        homeZAxis();
                        return;
                    }
                } else {
                    empty_scan_count = 0;  // Reset empty scan count if object found
                }
            }
            
            // Rotate the turntable for the full layer
            for(int loop_cont = 0; loop_cont < steps_per_rotation_for_motor; loop_cont++) {
                bool buttonReading = digitalRead(button);
                if (buttonReading != lastButtonState) {
                    lastDebounceTime = millis();
                }
                
                if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
                    if (buttonReading != buttonState) {
                        buttonState = buttonReading;
                        if (buttonState == LOW) { // Button pressed to stop
                            scan = 0;
                            digitalWrite(enable_z, HIGH);
                            digitalWrite(enable_r, HIGH);
                            Serial.println("Scan stopped.");
                            homeZAxis();
                            return;  // Exit the loop() function
                        }
                    }
                }
                lastButtonState = buttonReading;
                
                sensor_error = false;
                getDistance();
                
                if (!sensor_error) {
                    // Rotate the turntable
                    digitalWrite(enable_r, LOW);
                    digitalWrite(dir_r, LOW);
                    digitalWrite(step_r, HIGH);
                    delayMicroseconds(step_delay);
                    digitalWrite(step_r, LOW);
                    delayMicroseconds(step_delay);
                    angle = angle + RADIANS;
                    write_to_SD(x, y, z);
                }
            }
            
            if(scan == 0) return; // If scan was stopped during rotation
            
            // After the full rotation is completed, move the Z-axis
            while(z_loop < steps_z_height) {
                bool buttonReading = digitalRead(button);
                if (buttonReading != lastButtonState) {
                    lastDebounceTime = millis();
                }
                
                if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
                    if (buttonReading != buttonState) {
                        buttonState = buttonReading;
                        if (buttonState == LOW) { // Button pressed to stop
                            scan = 0;
                            digitalWrite(enable_z, HIGH);
                            digitalWrite(enable_r, HIGH);
                            Serial.println("Scan stopped.");
                            homeZAxis();
                            return;  // Exit the loop() function
                        }
                    }
                }
                lastButtonState = buttonReading;
                
                // Move the Z-axis step by step
                digitalWrite(enable_z, LOW);
                digitalWrite(dir_z, LOW); // Direction for Z-axis movement (upward)
                digitalWrite(step_z, HIGH);
                delayMicroseconds(step_delay);
                digitalWrite(step_z, LOW);
                delayMicroseconds(step_delay);
                z_loop++;
            }
            
            if(scan == 0) return; // If scan was stopped during Z-axis movement
            
            z = z + z_layer_height;  // Increment Z for the next layer
            z_loop = 0;  // Reset z_loop for next layer
        }
        else {
            digitalWrite(enable_z, HIGH);
            digitalWrite(enable_r, HIGH);
            scan = 0;
            Serial.println("Scan complete!");
            homeZAxis();
        }
    }
}

double getDistance() {
    analog = 0;
    measured_analog = 0;
    
    for (int aa = 0; aa < scan_amount; aa++) {
        measured_analog = analogRead(A0);
        delay(2);
        analog += measured_analog;
    }
    
    float raw_distance = analog / scan_amount;
    float voltage = raw_distance * 3.3 / 1023.0;  // Use the averaged raw distance
    
    float sensor_distance = 13 * pow(voltage, -1);  // Use datasheet formula
    
    if (sensor_distance < 4.0 || sensor_distance > 30.0) {
        sensor_error = true;
        return -1;
    }
    Serial.println(sensor_distance, 3);
    
    distance = abs(distance_to_center - sensor_distance);
    
    x = distance * cos(angle);
    y = distance * sin(angle);
    
    return distance;
}

void write_to_SD(float SDx, float SDy, float SDz) {
    file_values = SD.open(fileName, FILE_WRITE);
    if (file_values) {
        file_values.print(SDx, 3);
        file_values.print(",");
        file_values.print(SDy, 3);
        file_values.print(",");
        file_values.println(SDz, 3);
        file_values.close();
    }
    else {
        Serial.println("Error opening file!");
    }
}
