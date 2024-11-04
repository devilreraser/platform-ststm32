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

#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_subghz.h"
// Sub-GHz handle
SUBGHZ_HandleTypeDef hsubghz;


#define LED_RED PA9

// Queue handle and semaphore handle
QueueHandle_t uartQueue;
SemaphoreHandle_t uartSemaphore;

// Custom printf function that uses the semaphore for thread-safe access
void serial_printf(const char *format, ...) {
    // Wait for semaphore before accessing Serial
    if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
        char buffer[128];  // Buffer to hold the formatted string
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        Serial.print(buffer);  // Send the formatted string to Serial

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
    // HAL initialization
    HAL_Init();
    SystemClock_Config();
   // Enable the Sub-GHz radio
    __HAL_RCC_SUBGHZSPI_CLK_ENABLE();

    // Initialize the Sub-GHz radio
    HAL_SUBGHZ_Init(&hsubghz);



    Serial.begin(115200);
    while (!Serial);      // Wait for Serial to initialize
    Serial.println("UART Initialized.");

    // Create the semaphore for UART access
    uartSemaphore = xSemaphoreCreateMutex();
    if (uartSemaphore == NULL) {
        Serial.println("Error creating semaphore.");
        while (1);  // Halt if semaphore creation fails
    }

    // Initialize the queue to hold pointers to data buffers
    uartQueue = xQueueCreate(10, sizeof(char*));  // Queue holds up to 10 pointers
    if (uartQueue == NULL) {
        Serial.println("Error creating queue.");
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
    int availableBytes = Serial.available();

    // If there's no data, skip allocation
    if (availableBytes > 0) {
        // Allocate only the necessary memory to hold the data + 1 for null terminator
        char* buffer = (char*) pvPortMalloc((availableBytes + 1) * sizeof(char));
        if (buffer == NULL) {
            if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
                Serial.println("Memory allocation failed.");
                xSemaphoreGive(uartSemaphore);
            }
            delay(1000);  // Wait a second before retrying
            return;
        }

        // Read the available data into the allocated buffer
        for (int i = 0; i < availableBytes; i++) {
            buffer[i] = Serial.read();
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
    //vTaskDelay(pdMS_TO_TICKS(5000));
    delay(1000);


    // Use semaphore to make Serial output thread-safe
    if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
        Serial.print("Loop: ");
        Serial.print(millis());  // Print elapsed time in milliseconds
        Serial.println(" ms");
        Serial.print("portTICK_PERIOD_MS: ");
        Serial.println(portTICK_PERIOD_MS);  
        Serial.print("configTICK_RATE_HZ: ");
        Serial.println(configTICK_RATE_HZ);  
        Serial.print("configCPU_CLOCK_HZ: ");
        Serial.println(configCPU_CLOCK_HZ);  
        xSemaphoreGive(uartSemaphore);
    }
}
