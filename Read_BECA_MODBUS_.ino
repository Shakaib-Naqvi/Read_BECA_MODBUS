#include <HardwareSerial.h>

#define RS485_DE_RE_PIN 5  // DE and RE pins tied together and connected to GPIO 5
#define TX_PIN 17          // TX pin for UART1
#define RX_PIN 16          // RX pin for UART1

HardwareSerial RS485Serial(1);  // Use UART1 for RS485 communication

uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void setup() {
  Serial.begin(115200);
  RS485Serial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode
}

void loop() {
  // // Send request to read setpoint temperature from Beka thermostat
  // uint8_t frame[8];
  // frame[0] = 1;     // Slave address (assuming Beka thermostat address is 1)
  // frame[1] = 0x03;  // Function code (read holding registers)
  // frame[2] = 0x00;  // Starting address high byte (e.g., register address 0)
  // frame[3] = 0x05;  // Starting address low byte (e.g., register address 1 for setpoint)
  // frame[4] = 0x00;  // Quantity of registers high byte
  // frame[5] = 0x01;  // Quantity of registers low byte (read 1 register)

  // uint16_t crc = calculateCRC(frame, 6);
  // frame[6] = crc & 0xFF;         // CRC low byte
  // frame[7] = (crc >> 8) & 0xFF;  // CRC high byte

  // digitalWrite(RS485_DE_RE_PIN, HIGH);  // Set DE/RE pin high for transmit mode
  // RS485Serial.write(frame, 8);
  // RS485Serial.flush();                 // Ensure all data is sent
  // digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode

  // delay(500);  // Wait for the response

  // if (RS485Serial.available()) {
  //   uint8_t response[7];  // Assuming response size is fixed for 1 register (address + function code + data + CRC)
  //   RS485Serial.readBytes(response, 7);

  //   // Validate response CRC
  //   uint16_t receivedCRC = (response[6] << 8) | response[5];
  //   uint16_t calculatedCRC = calculateCRC(response, 5);
  //   if (receivedCRC == calculatedCRC) {
  //     // Process the setpoint temperature value
  //     uint16_t setpoint = (response[3] << 8) | response[4];  // Combine high byte and low byte to get the setpoint value
  //     float setpointFloat = (float)setpoint / 100.0;         // Convert to float if necessary (assuming setpoint is stored as an integer)

  //     Serial.print("Setpoint Temperature: ");
  //     Serial.println(setpointFloat);
  //   } else {
  //     Serial.println("CRC error!");
  //   }
  // }

  // delay(2000);  // Wait before sending the next r``````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````equest
  float setpoint;
  for (uint8_t i = 0; i < 10; i++) {
    setpoint = get_value_from_beca(i);
    setpoint = (float)setpoint / 10.0;
    Serial.print("Value Read From Register 0x000");
    Serial.print(i);
    Serial.print(" is: ");
    Serial.println(setpoint);
  }
  // setpoint = get_value_from_beca(3);
  // setpoint = (float)setpoint / 10.0;
  // Serial.print("Value Read From Register ");
  // Serial.print(3);
  // Serial.print(" is: ");
  // Serial.println(setpoint);
  delay(5000);
}



uint16_t get_value_from_beca(uint8_t start_register_address) {
  uint16_t value;
  float value_1 = 0;
  // Send request to read value from Beka thermostat

  uint8_t frame[8];
  frame[0] = 1;                              // Slave address (assuming Beka thermostat address is 1)
  frame[1] = 0x03;                           // Function code (read holding registers)
  frame[2] = 0x00;                           // Starting address high byte (e.g., register address 0)
  frame[2] = start_register_address >> 8;    // Starting address high byte
  frame[3] = start_register_address & 0xFF;  // Starting address low byte
  // frame[3] = 0xstartstart_register_address;  // Starting address low byte (e.g., register address 1 for setpoint)
  frame[4] = 0x00;  // Quantity of registers high byte
  frame[5] = 0x01;  // Quantity of registers low byte (read 1 register)

  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = crc & 0xFF;         // CRC low byte
  frame[7] = (crc >> 8) & 0xFF;  // CRC high byte

  digitalWrite(RS485_DE_RE_PIN, HIGH);  // Set DE/RE pin high for transmit mode
  RS485Serial.write(frame, 8);
  RS485Serial.flush();                 // Ensure all data is sent
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode

  delay(500);  // Wait for the response

  if (RS485Serial.available()) {
    uint8_t response[7];  // Assuming response size is fixed for 1 register (address + function code + data + CRC)
    RS485Serial.readBytes(response, 7);

    // Validate response CRC
    uint16_t receivedCRC = (response[6] << 8) | response[5];
    uint16_t calculatedCRC = calculateCRC(response, 5);
    if (receivedCRC == calculatedCRC) {
      // Process the  value
      uint16_t value = (response[3] << 8) | response[4];  // Combine high byte and low byte to get the  value
      // float value_1 = (float)value / 100.0;               // Convert to float if necessary (assuming setpoint is stored as an integer)
      return value;

      // Serial.print("Setpoint Temperature: ");
      // Serial.println(value_1);
    } else {  
      Serial.println("CRC error!");
    }
  }
}
