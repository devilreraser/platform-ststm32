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


// Custom printf function that uses the semaphore for thread-safe access
void serial_printf(const char *format, ...) {
    // Wait for semaphore before accessing Serial1
    if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
        char buffer[128];  // Buffer to hold the formatted string
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        Serial1.print(buffer);  // Send the formatted string to Serial1

        // Release the semaphore after printing
        xSemaphoreGive(uartSemaphore);
    }
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

// Task to dequeue and print data from the UART queue
void printTask(void *pvParameters) {
    char* receivedData;

    while (1) {
        // Wait to receive a pointer to the data from the queue
        if (xQueueReceive(uartQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
            serial_printf("Received: %s\r\n", receivedData);  // Print the received data as a string

            // Free the allocated memory after processing
            vPortFree(receivedData);
        }
    }
}

void setup()
{
    Serial1.begin(115200);
    while (!Serial1);      // Wait for Serial to initialize
    Serial1.println("UART Initialized.");

    // Create the semaphore for UART access
    uartSemaphore = xSemaphoreCreateMutex();
    if (uartSemaphore == NULL) {
        Serial1.println("Error creating semaphore.");
        while (1);  // Halt if semaphore creation fails
    }

    // Initialize the queue to hold pointers to data buffers
    uartQueue = xQueueCreate(10, sizeof(char*));  // Queue holds up to 10 pointers
    if (uartQueue == NULL) {
        Serial1.println("Error creating queue.");
        while (1);  // Halt if queue creation fails
    }

    // Create tasks for LED blinking and serial data handling
    xTaskCreate(blinkTask, "Blink Task", 128, NULL, 1, NULL);
    xTaskCreate(printTask, "Print Task", 256, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}

void loop()
{
    // Count the number of available bytes
    int availableBytes = Serial1.available();

    // If there's no data, skip allocation
    if (availableBytes > 0) {
        // Allocate only the necessary memory to hold the data + 1 for null terminator
        char* buffer = (char*) pvPortMalloc((availableBytes + 1) * sizeof(char));
        if (buffer == NULL) {
            if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
                Serial1.println("Memory allocation failed.");
                xSemaphoreGive(uartSemaphore);
            }
            delay(1000);  // Wait a second before retrying
            return;
        }

        // Read the available data into the allocated buffer
        for (int i = 0; i < availableBytes; i++) {
            buffer[i] = Serial1.read();
        }

        // Null-terminate the string
        buffer[availableBytes] = '\0';

        // Send the buffer pointer to the queue
        if (xQueueSend(uartQueue, &buffer, portMAX_DELAY) != pdTRUE) {
            // If sending fails, free the allocated memory
            vPortFree(buffer);
        }
    }

    // Wait for one second before reading the next batch of data
    delay(1000);

    // Use semaphore to make Serial1 output thread-safe
    if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
        Serial1.print("Loop: ");
        Serial1.print(millis());  // Print elapsed time in milliseconds
        Serial1.println(" ms");   // Print elapsed time in milliseconds
        xSemaphoreGive(uartSemaphore);
    }
}
