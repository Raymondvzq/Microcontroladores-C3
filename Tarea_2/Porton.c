/* 
 * Sistema de Control para Portón Automatizado
 * Control vía WiFi con comunicación MQTT
 * Publica estados y recibe comandos remotos
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "protocol_examples_common.h"
#include "mqtt_client.h"
#include "driver/gpio.h"

// =============================================
// DEFINICIONES Y CONFIGURACIÓN
// =============================================

// Estados del portón
typedef enum {
    ESTADO_CONFIG_INICIAL = 0,
    ESTADO_CERRANDO,
    ESTADO_ABRIENDO,
    ESTADO_ABIERTO,
    ESTADO_CERRADO,
    ESTADO_ERROR,
    ESTADO_PARADO
} estado_porton_t;

// Estados de las lámparas indicadoras
typedef enum {
    LAMPARA_APAGADA = 0,
    LAMPARA_MOVIMIENTO,
    LAMPARA_ERROR,
    LAMPARA_PARADA
} estado_lampara_t;

// Configuración de GPIO
#define GPIO_LED_INDICADOR    2
#define TIEMPO_MOVIMIENTO_MS  3000  // Tiempo simulado de apertura/cierre

// Configuración MQTT
#define MQTT_BROKER_URI       "ws://broker.emqx.io:8083/mqtt"
#define MQTT_USERNAME         "easy-learning"
#define MQTT_PASSWORD         "demo-para-el-canal"
#define TOPICO_COMANDOS       "easy-learning/puerta/cmd"
#define TOPICO_ESTADOS        "easy-learning/puerta/status"

// Comandos MQTT reconocidos
#define COMANDO_ABRIR         "abrir el puerton del itla"
#define COMANDO_CERRAR        "cerrar el puerton del itla"  
#define COMANDO_PARAR         "parar la puerta esa"

// Etiqueta para logs
static const char *TAG = "SISTEMA_PORTON";

// =============================================
// ESTRUCTURAS DE DATOS
// =============================================

// Estructura para entradas/salidas del sistema
typedef struct {
    uint8_t sensor_apertura;     // Límite switch apertura
    uint8_t sensor_cierre;       // Límite switch cierre
    uint8_t motor_apertura;      // Control motor apertura
    uint8_t motor_cierre;        // Control motor cierre
    estado_lampara_t lampara;    // Estado lámpara indicadora
} io_sistema_t;

// Estructura del contexto de la aplicación
typedef struct {
    estado_porton_t estado_actual;
    estado_porton_t estado_siguiente;
    estado_porton_t estado_anterior;
    io_sistema_t entradas_salidas;
    esp_mqtt_client_handle_t cliente_mqtt;
    TimerHandle_t timer_monitoreo;
} contexto_app_t;

// Variable global del contexto
static contexto_app_t app_context;

// =============================================
// PROTOTIPOS DE FUNCIONES
// =============================================

// Inicialización del sistema
void inicializar_sistema(void);
void configurar_gpio(void);

// MQTT
void iniciar_cliente_mqtt(void);
void manejar_evento_mqtt(void *handler_args, esp_event_base_t base, 
                         int32_t event_id, void *event_data);
void publicar_estado_mqtt(const char *estado);

// Máquina de estados
void actualizar_maquina_estados(void);
void ejecutar_transicion_estado(estado_porton_t nuevo_estado);

// Tareas y timers
void tarea_control_lampara(void *parametros);
void callback_timer_monitoreo(TimerHandle_t timer);

// =============================================
// IMPLEMENTACIÓN DE FUNCIONES
// =============================================

void inicializar_sistema(void) {
    ESP_LOGI(TAG, "Iniciando sistema de control de portón...");
    
    // Inicializar contexto
    memset(&app_context, 0, sizeof(app_context));
    app_context.estado_actual = ESTADO_CONFIG_INICIAL;
    app_context.estado_siguiente = ESTADO_CONFIG_INICIAL;
    app_context.estado_anterior = ESTADO_CONFIG_INICIAL;
}

void configurar_gpio(void) {
    ESP_LOGI(TAG, "Configurando GPIO...");
    
    gpio_reset_pin(GPIO_LED_INDICADOR);
    gpio_set_direction(GPIO_LED_INDICADOR, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED_INDICADOR, 0);
}

void manejar_evento_mqtt(void *handler_args, esp_event_base_t base, 
                        int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t evento = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "✅ Conectado exitosamente al broker MQTT");
            esp_mqtt_client_subscribe(app_context.cliente_mqtt, TOPICO_COMANDOS, 0);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "⚠️ Desconectado del broker MQTT");
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Comando recibido: %.*s", evento->data_len, evento->data);
            
            // Procesar comandos recibidos
            if (strncmp(evento->data, COMANDO_ABRIR, evento->data_len) == 0) {
                ESP_LOGI(TAG, "Ejecutando comando: ABRIR");
                ejecutar_transicion_estado(ESTADO_ABRIENDO);
            } 
            else if (strncmp(evento->data, COMANDO_CERRAR, evento->data_len) == 0) {
                ESP_LOGI(TAG, "Ejecutando comando: CERRAR");
                ejecutar_transicion_estado(ESTADO_CERRANDO);
            } 
            else if (strncmp(evento->data, COMANDO_PARAR, evento->data_len) == 0) {
                ESP_LOGI(TAG, "Ejecutando comando: PARAR");
                ejecutar_transicion_estado(ESTADO_PARADO);
            }
            else {
                ESP_LOGW(TAG, "Comando no reconocido");
            }
            break;
            
        default:
            break;
    }
}

void iniciar_cliente_mqtt(void) {
    ESP_LOGI(TAG, "Inicializando cliente MQTT...");
    
    esp_mqtt_client_config_t config_mqtt = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
        },
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication.password = MQTT_PASSWORD
        }
    };
    
    app_context.cliente_mqtt = esp_mqtt_client_init(&config_mqtt);
    esp_mqtt_client_register_event(app_context.cliente_mqtt, ESP_EVENT_ANY_ID, 
                                   manejar_evento_mqtt, NULL);
    esp_mqtt_client_start(app_context.cliente_mqtt);
}

void publicar_estado_mqtt(const char *estado) {
    if (app_context.cliente_mqtt) {
        char payload[64];
        snprintf(payload, sizeof(payload), "{\"estado\":\"%s\"}", estado);
        
        esp_mqtt_client_publish(app_context.cliente_mqtt, TOPICO_ESTADOS, 
                               payload, 0, 0, 0);
        ESP_LOGI(TAG, "Estado publicado: %s", estado);
    }
}

void ejecutar_transicion_estado(estado_porton_t nuevo_estado) {
    app_context.estado_siguiente = nuevo_estado;
}

void actualizar_maquina_estados(void) {
    // Guardar estado anterior antes de la transición
    app_context.estado_anterior = app_context.estado_actual;
    
    // Ejecutar transición de estado
    switch (app_context.estado_siguiente) {
        case ESTADO_CONFIG_INICIAL:
            ESP_LOGI(TAG, "🔄 Estado: CONFIGURACIÓN INICIAL");
            app_context.entradas_salidas.lampara = LAMPARA_APAGADA;
            app_context.estado_actual = ESTADO_CONFIG_INICIAL;
            publicar_estado_mqtt("configuracion_inicial");
            ejecutar_transicion_estado(ESTADO_CERRADO);
            break;
            
        case ESTADO_ABRIENDO:
            ESP_LOGI(TAG, "🚪 Estado: ABRIENDO (Anterior: %d)", app_context.estado_anterior);
            app_context.entradas_salidas.motor_apertura = 1;
            app_context.entradas_salidas.motor_cierre = 0;
            app_context.entradas_salidas.lampara = LAMPARA_MOVIMIENTO;
            gpio_set_level(GPIO_LED_INDICADOR, 1);
            app_context.estado_actual = ESTADO_ABRIENDO;
            publicar_estado_mqtt("abriendo");
            
            // Simular tiempo de apertura
            vTaskDelay(pdMS_TO_TICKS(TIEMPO_MOVIMIENTO_MS));
            ejecutar_transicion_estado(ESTADO_ABIERTO);
            break;
            
        case ESTADO_ABIERTO:
            ESP_LOGI(TAG, "✅ Estado: ABIERTO (Anterior: %d)", app_context.estado_anterior);
            app_context.entradas_salidas.motor_apertura = 0;
            app_context.entradas_salidas.motor_cierre = 0;
            app_context.entradas_salidas.lampara = LAMPARA_APAGADA;
            gpio_set_level(GPIO_LED_INDICADOR, 0);
            app_context.estado_actual = ESTADO_ABIERTO;
            publicar_estado_mqtt("abierto");
            break;
            
        case ESTADO_CERRANDO:
            ESP_LOGI(TAG, "🚪 Estado: CERRANDO (Anterior: %d)", app_context.estado_anterior);
            app_context.entradas_salidas.motor_apertura = 0;
            app_context.entradas_salidas.motor_cierre = 1;
            app_context.entradas_salidas.lampara = LAMPARA_MOVIMIENTO;
            gpio_set_level(GPIO_LED_INDICADOR, 1);
            app_context.estado_actual = ESTADO_CERRANDO;
            publicar_estado_mqtt("cerrando");
            
            // Simular tiempo de cierre
            vTaskDelay(pdMS_TO_TICKS(TIEMPO_MOVIMIENTO_MS));
            ejecutar_transicion_estado(ESTADO_CERRADO);
            break;
            
        case ESTADO_CERRADO:
            ESP_LOGI(TAG, "🔒 Estado: CERRADO (Anterior: %d)", app_context.estado_anterior);
            app_context.entradas_salidas.motor_apertura = 0;
            app_context.entradas_salidas.motor_cierre = 0;
            app_context.entradas_salidas.lampara = LAMPARA_APAGADA;
            gpio_set_level(GPIO_LED_INDICADOR, 0);
            app_context.estado_actual = ESTADO_CERRADO;
            publicar_estado_mqtt("cerrado");
            break;
            
        case ESTADO_PARADO:
            ESP_LOGI(TAG, "⏸️ Estado: PARADO (Anterior: %d)", app_context.estado_anterior);
            app_context.entradas_salidas.motor_apertura = 0;
            app_context.entradas_salidas.motor_cierre = 0;
            app_context.entradas_salidas.lampara = LAMPARA_PARADA;
            app_context.estado_actual = ESTADO_PARADO;
            publicar_estado_mqtt("parado");
            break;
            
        case ESTADO_ERROR:
            ESP_LOGE(TAG, "❌ Estado: ERROR");
            app_context.entradas_salidas.motor_apertura = 0;
            app_context.entradas_salidas.motor_cierre = 0;
            app_context.entradas_salidas.lampara = LAMPARA_ERROR;
            app_context.estado_actual = ESTADO_ERROR;
            publicar_estado_mqtt("error");
            break;
            
        default:
            ESP_LOGE(TAG, "Estado no reconocido: %d", app_context.estado_siguiente);
            break;
    }
}

void tarea_control_lampara(void *parametros) {
    ESP_LOGI(TAG, "Tarea de control de lámpara iniciada");
    
    while (1) {
        switch (app_context.entradas_salidas.lampara) {
            case LAMPARA_APAGADA:
                gpio_set_level(GPIO_LED_INDICADOR, 0);
                break;
                
            case LAMPARA_MOVIMIENTO:
                // El LED se maneja en la máquina de estados principal
                break;
                
            case LAMPARA_ERROR:
                // LED parpadeo rápido para error
                gpio_set_level(GPIO_LED_INDICADOR, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(GPIO_LED_INDICADOR, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
                
            case LAMPARA_PARADA:
                // LED parpadeo lento para parada
                gpio_set_level(GPIO_LED_INDICADOR, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(GPIO_LED_INDICADOR, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void callback_timer_monitoreo(TimerHandle_t timer) {
    // Monitoreo del sistema cada 50ms
    ESP_LOGD(TAG, "Sistema activo - Estado actual: %d", app_context.estado_actual);
}

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Iniciando aplicación principal...");
    
    // Inicialización del sistema
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    
    inicializar_sistema();
    configurar_gpio();
    iniciar_cliente_mqtt();
    
    // Crear timer de monitoreo
    app_context.timer_monitoreo = xTimerCreate(
        "TimerMonitoreo", 
        pdMS_TO_TICKS(50), 
        pdTRUE, 
        NULL, 
        callback_timer_monitoreo
    );
    xTimerStart(app_context.timer_monitoreo, 0);
    
    // Crear tarea para control de lámpara
    xTaskCreate(
        tarea_control_lampara, 
        "ControlLampara", 
        2048, 
        NULL, 
        5, 
        NULL
    );
    
    ESP_LOGI(TAG, "Sistema inicializado correctamente");
    
    // Bucle principal de la aplicación
    while (1) {
        actualizar_maquina_estados();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}