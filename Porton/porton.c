#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>

// ===== ESP-IDF core =====
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/gpio.h"

// ================== LOG TAG ==================
static const char *TAG = "PUERTA_MQTT";

// ================== WIFI ==================
#define WIFI_SSID      "Docentes_Administrativos"
#define WIFI_PASS      "Adm1N2584km"

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_event_group;

// ================== Definición de estados ==================
#define ESTADO_INICIAL     0
#define ESTADO_ERROR       1
#define ESTADO_ABRIENDO    2
#define ESTADO_ABIERTO     3
#define ESTADO_CERRANDO    4
#define ESTADO_CERRADO     5
#define ESTADO_DETENIDO    6
#define ESTADO_DESCONOCIDO 7

// ================== Definición de señales de E/S ==================
#define LM_ACTIVO    1
#define LM_NOACTIVO  0
#define MOTOR_OFF    0
#define MOTOR_ON     1
#define LAMP_OFF     0
#define LAMP_ON      1

// ================== Pines GPIO ==================
#define PIN_LSC        GPIO_NUM_12   // limit switch puerta cerrada
#define PIN_LSA        GPIO_NUM_13   // limit switch puerta abierta
#define PIN_MOTOR_A    GPIO_NUM_14   // motor de apertura
#define PIN_MOTOR_C    GPIO_NUM_27   // motor de cierre
#define PIN_LAMP       GPIO_NUM_26   // lámpara

// ================== MQTT ==================
#define MQTT_URI          "mqtt://broker.emqx.io:1883"
#define MQTT_USER         "Francisco"
#define MQTT_PASS_MQTT    "12345"
#define MQTT_TOPIC_CMD    "puerta/cmd"
#define MQTT_TOPIC_STATE  "puerta/state"

static esp_mqtt_client_handle_t mqtt_client = NULL;

// Comandos que llegan por MQTT
typedef enum {
    CMD_NONE = 0,
    CMD_ABRIR,
    CMD_CERRAR,
    CMD_STOP
} puerta_cmd_t;

static volatile puerta_cmd_t cmd_mqtt = CMD_NONE;

// ================== Variables de estado ==================
static int ESTADO_SIGUIENTE = ESTADO_INICIAL;
static int ESTADO_ANTERIOR  = ESTADO_INICIAL;
static int ESTADO_ACTUAL    = ESTADO_INICIAL;

// ================== Prototipos ==================
int Func_ESTADO_INICIAL(void);
int Func_ESTADO_ERROR(void);
int Func_ESTADO_ABRIENDO(void);
int Func_ESTADO_ABIERTO(void);
int Func_ESTADO_CERRANDO(void);
int Func_ESTADO_CERRADO(void);
int Func_ESTADO_DETENIDO(void);
int Func_ESTADO_DESCONOCIDO(void);
void Task_Maquina_Estados(void *pvParameters);

static void mqtt_app_start(void);
static void wifi_init_sta(void);

// ================== Funciones de E/S ==================
// Asumimos: switch normalmente abierto a GND con pull-up.
// 0 = activo (pulsado), 1 = no activo.
static inline int leer_switch(gpio_num_t pin) {
    int raw = gpio_get_level(pin);
    return (raw == 0) ? LM_ACTIVO : LM_NOACTIVO;
}

static inline void controlar_salida(gpio_num_t pin, int valor) {
    gpio_set_level(pin, valor);
}

// ================== Helper: publicar estado por MQTT ==================
static void publicar_estado_mqtt(void)
{
    if (mqtt_client == NULL) {
        return;
    }
    const char *estado_str = "desconocido";
    switch (ESTADO_ACTUAL) {
        case ESTADO_INICIAL:     estado_str = "inicial";     break;
        case ESTADO_ERROR:       estado_str = "error";       break;
        case ESTADO_ABRIENDO:    estado_str = "abriendo";    break;
        case ESTADO_ABIERTO:     estado_str = "abierto";     break;
        case ESTADO_CERRANDO:    estado_str = "cerrando";    break;
        case ESTADO_CERRADO:     estado_str = "cerrado";     break;
        case ESTADO_DETENIDO:    estado_str = "detenido";    break;
        case ESTADO_DESCONOCIDO: estado_str = "desconocido"; break;
        default:                 estado_str = "error";       break;
    }
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_STATE, estado_str, 0, 1, 0);
}

// ================== MQTT: manejo de errores ==================
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// ================== MQTT: event handler ==================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        mqtt_client = client;

        // Suscripción al tópico de comandos
        msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_CMD, 0);
        ESP_LOGI(TAG, "Suscrito a %s, msg_id=%d", MQTT_TOPIC_CMD, msg_id);

        // Publicar estado actual
        publicar_estado_mqtt();
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_client = NULL;
        break;

    case MQTT_EVENT_DATA: {
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");

        // Copiamos tópico y payload a buffers terminados en '\0'
        char topic[64];
        int topic_len = event->topic_len < (int)sizeof(topic)-1 ?
                        event->topic_len : (int)sizeof(topic)-1;
        memcpy(topic, event->topic, topic_len);
        topic[topic_len] = '\0';

        char data[128];
        int data_len = event->data_len < (int)sizeof(data)-1 ?
                       event->data_len : (int)sizeof(data)-1;
        memcpy(data, event->data, data_len);
        data[data_len] = '\0';

        ESP_LOGI(TAG, "TOPIC=%s, DATA=%s", topic, data);

        if (strcmp(topic, MQTT_TOPIC_CMD) == 0) {
            // Acepta texto plano ("ABRIR") y JSON ({"msg":"ABRIR"})
            if (strstr(data, "ABRIR") != NULL || strstr(data, "OPEN") != NULL) {
                cmd_mqtt = CMD_ABRIR;
                ESP_LOGI(TAG, "Comando MQTT: ABRIR");
            } else if (strstr(data, "CERRAR") != NULL || strstr(data, "CLOSE") != NULL) {
                cmd_mqtt = CMD_CERRAR;
                ESP_LOGI(TAG, "Comando MQTT: CERRAR");
            } else if (strstr(data, "STOP") != NULL || strstr(data, "DETENER") != NULL) {
                cmd_mqtt = CMD_STOP;
                ESP_LOGI(TAG, "Comando MQTT: STOP");
            } else {
                ESP_LOGW(TAG, "Comando desconocido: %s", data);
            }
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",
                                 event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)",
                     strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;

    default:
        ESP_LOGI(TAG, "Other MQTT event id:%d", event->event_id);
        break;
    }
}

// ================== MQTT: start ==================
static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS_MQTT,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// ================== WIFI: event handler ==================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi desconectado, reconectando...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi conectado, IP obtenida");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ================== WIFI: init STA ==================
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         NULL) );
    ESP_ERROR_CHECK( esp_event_handler_instance_register(IP_EVENT,
                                                         IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         NULL) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    // Esperar conexión
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Conectado a AP SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "No se pudo conectar a AP");
    }
}

// ================== FSM: implementación de estados ==================

// INICIAL: solo apaga todo y salta a DESCONOCIDO (sin imprimir)
int Func_ESTADO_INICIAL(void) {
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL   = ESTADO_INICIAL;

    controlar_salida(PIN_MOTOR_A, MOTOR_OFF);
    controlar_salida(PIN_MOTOR_C, MOTOR_OFF);
    controlar_salida(PIN_LAMP,   LAMP_OFF);

    int lsa = leer_switch(PIN_LSA);
    int lsc = leer_switch(PIN_LSC);

    // Si ambos switches activos = algo raro → error
    if (lsa == LM_ACTIVO && lsc == LM_ACTIVO) {
        return ESTADO_ERROR;
    }

    // No queremos que la puerta se mueva sola al arrancar:
    // pasamos a DESCONOCIDO en silencio.
    return ESTADO_DESCONOCIDO;
}

int Func_ESTADO_ERROR(void) {
    if (ESTADO_ACTUAL != ESTADO_ERROR) {
        ESTADO_ANTERIOR = ESTADO_ACTUAL;
        ESTADO_ACTUAL   = ESTADO_ERROR;
        printf("[FSM] Estado Error!\n");
        publicar_estado_mqtt();
    }

    // Parpadeo sencillo de la lámpara
    controlar_salida(PIN_LAMP, LAMP_ON);
    vTaskDelay(pdMS_TO_TICKS(300));
    controlar_salida(PIN_LAMP, LAMP_OFF);
    vTaskDelay(pdMS_TO_TICKS(300));
    return ESTADO_INICIAL;
}

// ===== ESTADO ABRIENDO: motor ON hasta activar LSA =====
int Func_ESTADO_ABRIENDO(void) {
    if (ESTADO_ACTUAL != ESTADO_ABRIENDO) {
        ESTADO_ANTERIOR = ESTADO_ACTUAL;
        ESTADO_ACTUAL   = ESTADO_ABRIENDO;
        printf("[FSM] Estado Abriendo\n");
        publicar_estado_mqtt();
    }

    // Motor solo en sentido de apertura
    controlar_salida(PIN_MOTOR_C, MOTOR_OFF);
    controlar_salida(PIN_MOTOR_A, MOTOR_ON);

    int lsa = leer_switch(PIN_LSA);  // FIN DE CARRERA DE PUERTA ABIERTA

    if (lsa == LM_ACTIVO) {
        // Llegó a fin de carrera de apertura
        controlar_salida(PIN_MOTOR_A, MOTOR_OFF);
        printf("[FSM] Llegó a fin de carrera ABIERTO\n");
        publicar_estado_mqtt();
        return ESTADO_ABIERTO;
    }

    // No imprimimos en cada ciclo, solo mantenemos motor hasta fin de carrera
    return ESTADO_ABRIENDO;  // seguimos abriendo
}

int Func_ESTADO_ABIERTO(void) {
    if (ESTADO_ACTUAL != ESTADO_ABIERTO) {
        ESTADO_ANTERIOR = ESTADO_ACTUAL;
        ESTADO_ACTUAL   = ESTADO_ABIERTO;
        printf("[FSM] Estado Abierto\n");
        publicar_estado_mqtt();
    }

    // En este estado esperamos comando CERRAR por MQTT
    vTaskDelay(pdMS_TO_TICKS(500));
    if (cmd_mqtt == CMD_CERRAR) {
        cmd_mqtt = CMD_NONE;
        return ESTADO_CERRANDO;
    }
    return ESTADO_ABIERTO;
}

// ===== ESTADO CERRANDO: motor ON hasta LSC o timeout 5s =====
int Func_ESTADO_CERRANDO(void) {
    int primera_vez = (ESTADO_ACTUAL != ESTADO_CERRANDO);

    if (primera_vez) {
        ESTADO_ANTERIOR = ESTADO_ACTUAL;
        ESTADO_ACTUAL   = ESTADO_CERRANDO;
        printf("[FSM] Estado Cerrando\n");
        publicar_estado_mqtt();
    }

    // Timer estático para este estado
    static TickType_t inicio_cerrando = 0;
    if (primera_vez) {
        inicio_cerrando = xTaskGetTickCount();
        ESP_LOGI(TAG, "[FSM] Inicio CERRANDO, arrancamos timer");
    }

    // Motor solo en sentido de cierre
    controlar_salida(PIN_MOTOR_A, MOTOR_OFF);
    controlar_salida(PIN_MOTOR_C, MOTOR_ON);

    int lsc = leer_switch(PIN_LSC);  // FIN DE CARRERA DE PUERTA CERRADA
    TickType_t ahora = xTaskGetTickCount();
    uint32_t elapsed_ms = (ahora - inicio_cerrando) * portTICK_PERIOD_MS;

    // 1) Caso OK: llegó a LSC
    if (lsc == LM_ACTIVO) {
        controlar_salida(PIN_MOTOR_C, MOTOR_OFF);
        printf("[FSM] Llegó a fin de carrera CERRADO\n");
        publicar_estado_mqtt();
        return ESTADO_CERRADO;
    }

    // 2) Caso ERROR: pasó más de 5 segundos y LSC sigue NO activo
    if (elapsed_ms > 5000 && lsc != LM_ACTIVO) {
        controlar_salida(PIN_MOTOR_C, MOTOR_OFF);
        printf("[FSM] ERROR: Timeout cerrando (%" PRIu32 " ms, LSC=%d)\n", elapsed_ms, lsc);
        publicar_estado_mqtt();
        return ESTADO_ERROR;
    }

    // 3) Seguimos cerrando (sin spam de prints)
    return ESTADO_CERRANDO;  // seguimos cerrando
}

int Func_ESTADO_CERRADO(void) {
    if (ESTADO_ACTUAL != ESTADO_CERRADO) {
        ESTADO_ANTERIOR = ESTADO_ACTUAL;
        ESTADO_ACTUAL   = ESTADO_CERRADO;
        printf("[FSM] Estado Cerrado\n");
        publicar_estado_mqtt();
    }

    // En este estado esperamos comando ABRIR
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (cmd_mqtt == CMD_ABRIR) {
        cmd_mqtt = CMD_NONE;
        return ESTADO_ABRIENDO;
    }
    return ESTADO_CERRADO;
}

int Func_ESTADO_DETENIDO(void) {
    if (ESTADO_ACTUAL != ESTADO_DETENIDO) {
        ESTADO_ANTERIOR = ESTADO_ACTUAL;
        ESTADO_ACTUAL   = ESTADO_DETENIDO;
        printf("[FSM] Estado Detenido\n");
        publicar_estado_mqtt();
    }

    controlar_salida(PIN_MOTOR_A, MOTOR_OFF);
    controlar_salida(PIN_MOTOR_C, MOTOR_OFF);

    // Desde detenido solo salimos por comando ABRIR o CERRAR
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (cmd_mqtt == CMD_ABRIR) {
        cmd_mqtt = CMD_NONE;
        return ESTADO_ABRIENDO;
    }
    if (cmd_mqtt == CMD_CERRAR) {
        cmd_mqtt = CMD_NONE;
        return ESTADO_CERRANDO;
    }
    return ESTADO_DETENIDO;
}

// ===== ESTADO DESCONOCIDO: NO imprime y NO mueve motor hasta comando MQTT =====
int Func_ESTADO_DESCONOCIDO(void) {
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL   = ESTADO_DESCONOCIDO;

    // Motores SIEMPRE apagados en este estado
    controlar_salida(PIN_MOTOR_A, MOTOR_OFF);
    controlar_salida(PIN_MOTOR_C, MOTOR_OFF);

    // Nada de prints aquí, como pediste.
    // Solo esperamos comandos MQTT.
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (cmd_mqtt == CMD_ABRIR) {
        cmd_mqtt = CMD_NONE;
        return ESTADO_ABRIENDO;
    }
    if (cmd_mqtt == CMD_CERRAR) {
        cmd_mqtt = CMD_NONE;
        return ESTADO_CERRANDO;
    }

    // Sin comando, nos quedamos aquí en silencio.
    return ESTADO_DESCONOCIDO;
}

// ================== Tarea de la máquina de estados ==================
void Task_Maquina_Estados(void *pvParameters) {
    while (1) {
        // Comando STOP global: manda al estado DETENIDO
        if (cmd_mqtt == CMD_STOP) {
            cmd_mqtt = CMD_NONE;
            ESTADO_SIGUIENTE = ESTADO_DETENIDO;
        }

        switch (ESTADO_SIGUIENTE) {
            case ESTADO_INICIAL:
                ESTADO_SIGUIENTE = Func_ESTADO_INICIAL();
                break;
            case ESTADO_ERROR:
                ESTADO_SIGUIENTE = Func_ESTADO_ERROR();
                break;
            case ESTADO_ABRIENDO:
                ESTADO_SIGUIENTE = Func_ESTADO_ABRIENDO();
                break;
            case ESTADO_ABIERTO:
                ESTADO_SIGUIENTE = Func_ESTADO_ABIERTO();
                break;
            case ESTADO_CERRANDO:
                ESTADO_SIGUIENTE = Func_ESTADO_CERRANDO();
                break;
            case ESTADO_CERRADO:
                ESTADO_SIGUIENTE = Func_ESTADO_CERRADO();
                break;
            case ESTADO_DETENIDO:
                ESTADO_SIGUIENTE = Func_ESTADO_DETENIDO();
                break;
            case ESTADO_DESCONOCIDO:
                ESTADO_SIGUIENTE = Func_ESTADO_DESCONOCIDO();
                break;
            default:
                ESTADO_SIGUIENTE = ESTADO_ERROR;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // ciclo cada ~50 ms
    }
}

// ================== app_main ==================
void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes",
             esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializar pila de red y loop de eventos
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    // WiFi STA y conexión al AP
    wifi_init_sta();

    // Iniciar MQTT (requiere WiFi conectado)
    mqtt_app_start();

    // Configurar GPIOs
    gpio_config_t io_conf = {0};

    // Entradas: LSC y LSA (con pull-up)
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_LSC) | (1ULL<<PIN_LSA);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Salidas: motores y lámpara
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_MOTOR_A) |
                           (1ULL<<PIN_MOTOR_C) |
                           (1ULL<<PIN_LAMP);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Crear la tarea de la FSM
    xTaskCreate(
        Task_Maquina_Estados,
        "Maquina_Estados",
        4096,      // tamaño de stack
        NULL,
        5,         // prioridad
        NULL
    );
}
