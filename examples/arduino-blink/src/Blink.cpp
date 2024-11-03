/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>


HardwareSerial Serial1(PB7, PB6); // RX, TX pins

#define LED_RED PA9

// Queue handle and semaphore handle
QueueHandle_t uartQueue;
SemaphoreHandle_t uartSemaphore;


// Custom printf function that prints to Serial1
void serial_printf(const char *format, ...) {
    char buffer[128];   // Buffer to hold the formatted string
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial1.print(buffer);  // Send the formatted string to Serial1
}

void blinkTask(void *pvParameters) {
    const int ledPin = LED_RED;

    pinMode(ledPin, OUTPUT);

    while (1) {
        digitalWrite(ledPin, HIGH);
        serial_printf("LED On\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));

        digitalWrite(ledPin, LOW);
        serial_printf("LED Off\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void setup()
{
  Serial1.begin(115200);
  while (!Serial1);      // Wait for Serial to initialize
  Serial1.println("UART Initialized.");



  xTaskCreate(blinkTask, "Blink Task", 128, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop()
{
  // Check if data is available to read
  if (Serial1.available() > 0) {
      char received = Serial1.read();  // Read the incoming byte
      Serial1.print("Received: ");
      Serial1.println(received);       // Print the received byte
  }
  delay(1000);
  Serial1.print("Loop: ");
  Serial1.print(millis()); // Print elapsed time in seconds
  Serial1.println(" ms"); // Print elapsed time in seconds
}
