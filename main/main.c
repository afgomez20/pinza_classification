#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRtos.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/semphr.h"
#include "math.h"
#include "custom_servo.h"
#include "model_data.h"

//static const char* tag = "Main";   
#define STACK_SIZE 2048
int adc_value = 0;
int servo0_pin = 12;//gpio 12
int servo1_pin = 14;//gpio 14
int accion = 0;
int giro = 0; //0 hacia la derecha 1 hacia la izquierda
xSemaphoreHandle GlobalKey = 0;
int datos[300];
float datos_norm[300];
float datos_norm[300];
float feature[13];
float ventana1[50];
float ventana2[50];
float ventana3[50];
float ventana4[50];
float ventana5[50];
float ventana6[50];

esp_err_t set_adc(void);
esp_err_t create_tasks(void);
void vTaskServo(void *pvParameters);
void vTaskData(void *pvParameters);
void vTaskB(void *pvParameters);
float calculate_min(float ventana[50]);
float calculate_max(float ventana[50]);

static void init_uart(void){

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_2,&uart_config);
}


void app_main(void)
{
    init_uart();
    add_servo(servo0_pin,0);//giro
    add_servo(servo1_pin,1);//pinza
    servo_setPosition(0,0);
    create_tasks();
    GlobalKey = xSemaphoreCreateBinary();

    while(1){
        /**adc_value = adc1_get_raw(ADC1_CHANNEL_4);
  
        printf("%i\n",adc_value);
        //ESP_LOGI(tag,"%i",adc_value);
        vTaskDelay(10/portTICK_PERIOD_MS);**/
        

        
    }
}



esp_err_t set_adc(void){
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);//gpio32
    adc1_config_width(ADC_WIDTH_BIT_12);
    return ESP_OK;
}

esp_err_t create_tasks(void)
{
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate(vTaskServo,
                "vTaskServo",
                STACK_SIZE,//tamaño de memoria
                &ucParameterToPass,
                2, //prioridad
                &xHandle);

    xTaskCreate(vTaskData,
                "vTaskData",
                STACK_SIZE,
                &ucParameterToPass,
                2, 
                &xHandle);
    return ESP_OK;
}

void vTaskServo(void *pvParameters)
{
    while (1)
    {
        if(xSemaphoreTake(GlobalKey,portMAX_DELAY)){
            if (accion == 0){//abrir mano
                servo_setPosition(0,1);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            else if(accion == 1){//cerrar mano
                servo_setPosition(20,1);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            else if(accion == 2 && giro == 0){//giro derecha
                servo_setPosition(60,0);//sujeto a cambios revisar
                giro = 1;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            else if(accion == 2 && giro == 1){//giro izquierda
                servo_setPosition(0,0);//sujeto a cambios revisar
                giro = 0;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        //xSemaphoreGive();
        //xSemaphoreTake
        //portMAX_DELAY
    }
}

void vTaskData(void *pvParameters)
{
    while (1)
    {
        for (int i = 0;i<=299;i++){
            adc_value = adc1_get_raw(ADC1_CHANNEL_4);
            printf("%i\n",adc_value);
            datos[i]=adc_value;
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        //media de los cuadrados
        float suma = 0;
        for (int i = 0;i<=299;i++){
            suma += pow(datos[i],2);
        }
        float media_cuadrada = suma/300;
        //Obtener el valor máximo y mínimo del arreglo
        int max=datos[0];
        for(int i=0; i<300; i++){
            if (datos[i] > max){
                max=datos[i];
            }
        }
        printf("El valor máximo es %.2i", max);
    
    
    
        int min=datos[0];
        for(int i=0; i<300; i++){
            if (datos[i] < min){
                min=datos[i];
            }
        }
        printf("El valor minimo es %.2i", min);

        //Filtrar 

        //normalizar
        for (int i = 0;i<=299;i++){
            datos_norm[i] = (datos[i]-min)/(datos[i]-max);
        }
        //feature
        for (int i = 0;i<=299;i++){
            if(i<50){
                ventana1[i] = datos_norm[i];
            }
            else if(i>=50 && i<100){
                ventana2[i-50] = datos_norm[i];
            }
            else if(i>=100 && i<150){
                ventana3[i-100] = datos_norm[i];
            }
            else if(i>=150 && i<200){
                ventana4[i-150] = datos_norm[i];
            }
             else if(i>=200 && i<250){
                ventana5[i-200] = datos_norm[i];
            }
            else if(i>=250 && i<300){
                ventana6[i-250] = datos_norm[i];
            }
        }
        
        float min1 = calculate_min(ventana1);
        feature[0] = min1;
        float max1 = calculate_max(ventana1);
        feature[1] = max1;
        float min2 = calculate_min(ventana2);
        feature[2] = min2;
        float max2 = calculate_max(ventana2);
        feature[3] = max2;
        float min3 = calculate_min(ventana3);
        feature[4] = min3;
        float max3 = calculate_max(ventana3);
        feature[5] = max3;
        float min4 = calculate_min(ventana4);
        feature[6] = min4;
        float max4 = calculate_max(ventana4);
        feature[7] = max4;
        float min5 = calculate_min(ventana5);
        feature[8] = min5;
        float max5 = calculate_max(ventana5);
        feature[9] = max5;
        float min6 = calculate_min(ventana6);
        feature[10] = min6;
        float max6 = calculate_max(ventana6);
        feature[11] = max6;
        feature[12] = sqrt(media_cuadrada);
        xSemaphoreGive(GlobalKey);

    }
}

float calculate_min(float ventana[50]){
    float min=ventana[0];
        for(int i=0; i<50; i++){
            if (ventana[i] < min){
                min=ventana[i];
            }
        }
    return min;
}
float calculate_max(float ventana[50]){
    float max=ventana[0];
        for(int i=0; i<50; i++){
            if (ventana[i] > max){
                max=ventana[i];
            }
        }
    return max;
}