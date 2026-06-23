#include <Arduino.h>

#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 256000

// Correct frame size: 4 header + 24 payload + 2 footer = 30 bytes
#define FRAME_SIZE 30

enum ParserState {
    FIND_HEADER_0,
    FIND_HEADER_1,
    FIND_HEADER_2,
    FIND_HEADER_3,
    READ_PAYLOAD
};

ParserState parser_state = FIND_HEADER_0;
uint8_t payload_idx = 0;
uint8_t frame_data[FRAME_SIZE] = {0};

uint8_t Multi_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};

// Sign-magnitude decode: bit15=sign, bits14:0=magnitude
int16_t parseRD03DSigned(uint8_t low, uint8_t high) {
    uint16_t value = ((uint16_t)high << 8) | low;
    bool is_positive = (value & 0x8000) != 0;
    int16_t magnitude = (int16_t)(value & 0x7FFF);
    return is_positive ? magnitude : -magnitude;
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\nInitializing RD-03D...");

    // MUST set buffer size BEFORE begin()
    Serial1.setRxBufferSize(512);
    Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

    delay(2000);

    Serial1.write(Multi_Target_Detection_CMD, sizeof(Multi_Target_Detection_CMD));
    delay(500);

    while (Serial1.available() > 0) Serial1.read();
    Serial.println("Ready. Multi-target mode active.");
}

void processValidatedFrame(uint8_t* frame) {
    // Target 1: bytes [4..11]
    int16_t t1_x     = parseRD03DSigned(frame[4],  frame[5]);
    int16_t t1_y     = parseRD03DSigned(frame[6],  frame[7]);
    int16_t t1_speed = parseRD03DSigned(frame[8],  frame[9]);
    uint16_t t1_res  = ((uint16_t)frame[11] << 8) | frame[10];

    // Target 2: bytes [12..19]
    int16_t t2_x     = parseRD03DSigned(frame[12], frame[13]);
    int16_t t2_y     = parseRD03DSigned(frame[14], frame[15]);
    int16_t t2_speed = parseRD03DSigned(frame[16], frame[17]);
    uint16_t t2_res  = ((uint16_t)frame[19] << 8) | frame[18];

    // Target 3: bytes [20..27]
    int16_t t3_x     = parseRD03DSigned(frame[20], frame[21]);
    int16_t t3_y     = parseRD03DSigned(frame[22], frame[23]);
    int16_t t3_speed = parseRD03DSigned(frame[24], frame[25]);
    uint16_t t3_res  = ((uint16_t)frame[27] << 8) | frame[26];

    // Active = resolution value is nonzero (radar is tracking this slot)
    int t1_active = (t1_res > 0) ? 1 : 0;
    int t2_active = (t2_res > 0) ? 1 : 0;
    int t3_active = (t3_res > 0) ? 1 : 0;

    float t1_dist_cm  = t1_active ? sqrt((float)t1_x*t1_x + (float)t1_y*t1_y) / 10.0f : 0.0f;
    float t1_angle    = t1_active ? atan2f(t1_y, t1_x) * 180.0f / PI : 0.0f;
    float t2_dist_cm  = t2_active ? sqrt((float)t2_x*t2_x + (float)t2_y*t2_y) / 10.0f : 0.0f;
    float t2_angle    = t2_active ? atan2f(t2_y, t2_x) * 180.0f / PI : 0.0f;
    float t3_dist_cm  = t3_active ? sqrt((float)t3_x*t3_x + (float)t3_y*t3_y) / 10.0f : 0.0f;
    float t3_angle    = t3_active ? atan2f(t3_y, t3_x) * 180.0f / PI : 0.0f;

    // CSV output: x,y,dist,angle,speed,active  (x3 targets)
    Serial.print(t1_x); Serial.print(",");
    Serial.print(t1_y); Serial.print(",");
    Serial.print(t1_dist_cm, 1); Serial.print(",");
    Serial.print(t1_angle, 1); Serial.print(",");
    Serial.print(t1_speed); Serial.print(",");
    Serial.print(t1_active); Serial.print(",");

    Serial.print(t2_x); Serial.print(",");
    Serial.print(t2_y); Serial.print(",");
    Serial.print(t2_dist_cm, 1); Serial.print(",");
    Serial.print(t2_angle, 1); Serial.print(",");
    Serial.print(t2_speed); Serial.print(",");
    Serial.print(t2_active); Serial.print(",");

    Serial.print(t3_x); Serial.print(",");
    Serial.print(t3_y); Serial.print(",");
    Serial.print(t3_dist_cm, 1); Serial.print(",");
    Serial.print(t3_angle, 1); Serial.print(",");
    Serial.print(t3_speed); Serial.print(",");
    Serial.println(t3_active);
}

void loop() {
    while (Serial1.available()) {
        uint8_t val = Serial1.read();

        switch (parser_state) {
            case FIND_HEADER_0:
                if (val == 0xAA) { frame_data[0] = val; parser_state = FIND_HEADER_1; }
                break;
            case FIND_HEADER_1:
                if (val == 0xFF) { frame_data[1] = val; parser_state = FIND_HEADER_2; }
                else parser_state = FIND_HEADER_0;
                break;
            case FIND_HEADER_2:
                if (val == 0x03) { frame_data[2] = val; parser_state = FIND_HEADER_3; }
                else parser_state = FIND_HEADER_0;
                break;
            case FIND_HEADER_3:
                if (val == 0x00) { frame_data[3] = val; payload_idx = 4; parser_state = READ_PAYLOAD; }
                else parser_state = FIND_HEADER_0;
                break;
            case READ_PAYLOAD:
                frame_data[payload_idx++] = val;
                if (payload_idx >= FRAME_SIZE) {
                    // Footer is at [28] and [29]
                    if (frame_data[28] == 0x55 && frame_data[29] == 0xCC) {
                        processValidatedFrame(frame_data);
                    }
                    parser_state = FIND_HEADER_0;
                }
                break;
        }
    }
}