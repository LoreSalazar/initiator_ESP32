#include <stdio.h>
#include "string.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define ESP_CHANNEL 1
//#define led 2

uint8_t data[8];

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xf4, 0x65, 0x0b, 0x46, 0x53, 0x7c}; //* define un arreglo de 6 bytes inicializado en cero, destinado a almacenar la dirección MAC de un peer (dispositivo remoto) para usar en ESP-NOW.

static const char *tag = "ESP-NOW";


static esp_err_t init_wifi(void){   //Inicialización del wifi que es necesario para poder trabajar con ESP-NOW

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();  //Declaración del wifi por defecto

    esp_netif_init();                   //Inicialización de NEFIT
    esp_event_loop_create_default();    //Inicialización del event loop
    nvs_flash_init();                   //Inicialización de NVS flash

    esp_wifi_init(&wifi_init_config);   //Inicializa el stack Wifi. Este paso prepara los controladores y estructuras internas del Wi-Fi.
    esp_wifi_set_mode(WIFI_MODE_STA);   //El modo estación es cuando el ESP32 se conecta a una red Wi-Fi existente, como si fuera un celular o laptop.
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);   //WIFI_STORAGE_FLASH: guarda la configuración (SSID, contraseña, etc.) en la flash (no volátil). También se puede usar WIFI_STORAGE_RAM si se requiere que se borre al reiniciar
    esp_wifi_start();                   //Comienza la actividad de wifi

    ESP_LOGI(tag, "wifi inicializado corectamente");

    return ESP_OK;
}

/*void recv_callback(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len){    //Función que se ejecuta cuando ya se recibieron los datos

    //Se va a transmitir lo que llega 
    ESP_LOGI(tag, "Datos recibidos: " MACSTR "%s", MAC2STR(esp_now_info->src_addr), data);    //MACSTR es un macro especial de ESP-IDF que se expande a "%02x:%02x:%02x:%02x:%02x:%02x" para imprimir direcciones MAC.
                                                                                              //Otro macro de ESP-IDF que convierte la MAC en 6 bytes (uint8_t[6]) en 6 argumentos que se ajustan a MACSTR
}*/

void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status){  //Función que se ejecuta cuando se terminan de transmitir los datos

    if(status == ESP_NOW_SEND_SUCCESS){
        ESP_LOGI(tag, "ESP-NOW datos enviados con exito");
    }else{
        ESP_LOGE(tag, "ESP-NOW fallo el envio de datos");
    }
}

static esp_err_t init_esp_now(void){   //Inicialización de ESP-NOW

    esp_now_init(); //Inicialización de esp-now
    //esp_now_register_recv_cb(recv_callback);  //Registra una función de callback que se ejecuta cuando se recibe un mensaje por ESP-NOW.
    esp_now_register_send_cb(send_callback);   //Registra una función de callback que se ejecuta cuando se recibe un mensaje por ESP-NOW.

    ESP_LOGI(tag, "Inicialización de ESP-NOW completada");

    return ESP_OK;
}

static esp_err_t register_peer(uint8_t *peer_addr){
   
    //Creación de un peer (Dispositivo Remoto)
    esp_now_peer_info_t esp_now_peer_info = {}; //Se declara e inicializa en cero una estructura esp_now_peer_info_t, que contiene la info necesaria para registrar un peer.

    memcpy(esp_now_peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);    //Copia la dirección MAC del peer a registrar dentro de la estructura.
    esp_now_peer_info.channel   = ESP_CHANNEL;  //Asigna el canal Wi-Fi en el que va a comunicarse con ese peer.
    esp_now_peer_info.ifidx     = ESP_IF_WIFI_STA;  //Define la interfaz Wi-Fi que se usa para ESP-NOW. ESP_IF_WIFI_STA = se usa la interfaz en modo estación, que es lo típico para ESP-NOW.

    esp_now_add_peer(&esp_now_peer_info);   //Llama a la API de ESP-IDF para registrar ese peer en el sistema. Esto es necesario antes de poder enviarle datos con esp_now_send().
   
    return ESP_OK;
}

//Función para enviar datos:
static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, uint8_t len){

    esp_now_send(peer_addr, data, len);   //Se usa esp_now_send() (la API de ESP-IDF) para enviar datos a un peer por ESP-NOW.
    return ESP_OK;
}

//esp_err_t led_init();

void app_main(void){

    ESP_ERROR_CHECK(init_wifi());   //Revisar los posibles errores que haya al inicializar el wifi
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(register_peer(peer_mac));

    //Envío de datos al responder

    while(1){
        
        data[0] = 0x7E;
        data[1] = 0xAB;
        data[2] = 0x11;
        data[3] = 0x3D;
        data[4] = 0x44;
        data[5] = 0xA4;
        data[6] = 0x73;
        data[7] = 0x31;

        esp_now_send_data(peer_mac, data, 8);
        vTaskDelay(100 / portTICK_PERIOD_MS);   //Uso de tasks para la creación de un retardo)
    }

}

/*esp_err_t led_init(){



    return ESP_OK;
}*/


/*ESP-NOW es un protocolo inalámbrico de Espressif que permite comunicación directa entre dispositivos 
(ESP32, ESP8266, etc.), sin necesidad de Wi-Fi tradicional o router. Es rápido y eficiente.*/

/** (Línea 17) La variable peer_mac tiene duración de almacenamiento estática, es decir:
        * Se guarda en memoria global (no en la pila).
        * Su valor se mantiene entre llamadas a la función donde esté (si está dentro de una función).
        * Si está fuera de cualquier función, su alcance se limita al archivo (ámbito local al archivo .c). 
    
    ESP_NOW_ETH_ALEN es un macro que vale 6, correspondiente al tamaño de una dirección MAC (6 bytes).   */

/*¿QUÉ SIGNIFICA USAR LA INTERFAZ EN MODO ESTACIÓN?

    Usar la interfaz en modo estación (STA) significa que tu ESP32 se comporta como un cliente Wi-Fi, 
    es decir, se conecta a una red Wi-Fi existente (como la de tu casa, oficina o el hotspot de tu celular), 
    en lugar de crear una red propia.
    
    Modo ---------- ¿Qué hace?
    WIFI_MODE_STA	El ESP32 se conecta a una red Wi-Fi existente. Es como un teléfono o laptop.
    WIFI_MODE_AP	El ESP32 crea su propia red Wi-Fi (Access Point) para que otros se conecten a él.
    WIFI_MODE_APSTA	Modo mixto: el ESP32 crea una red y a la vez se conecta a otra.
    WIFI_MODE_NULL	El Wi-Fi está apagado.
    
    Aunque ESP-NOW no requiere un router Wi-Fi, el ESP32 sigue usando su interfaz Wi-Fi física, y 
    debes decirle en qué "modo" está trabajando*/

