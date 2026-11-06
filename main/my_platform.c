#include <string.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include <uni.h>
#include <esp_log.h>
#include <esp_err.h>
#include "driver/ledc.h"
#include <math.h>

// Definição da GPIO utilizada no controle das placas
/* ─── Pinos de direção ─────────────────────────────────────────── */
#define IN1_PIN  GPIO_NUM_32    // Motor A pra frente
#define IN2_PIN  GPIO_NUM_33    // Motor A pra tras
#define IN3_PIN  GPIO_NUM_25    // Motor B pra frente
#define IN4_PIN  GPIO_NUM_26    // Motor B pra tras

/* ─── Pinos de velocidade (PWM) ────────────────────────────────────── */
#define ENA_PIN  GPIO_NUM_19   // Motor A
#define ENB_PIN  GPIO_NUM_18   // Motor B

// Definição dos parâmetros do PWM
/* ─── Configuração LEDC ────────────────────────────────────────── */
#define PWM_FREQ_HZ     10000                   // 10 kHz
#define PWM_RESOLUTION  LEDC_TIMER_10_BIT       // 1024 níveis (0–1023)

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE     // Fast PWM = Low Speed no ESP32

#define CHANNEL_A       LEDC_CHANNEL_0          // EN para motor A
#define CHANNEL_B       LEDC_CHANNEL_1          // EN para motor B

#define DEADZONE 20
#define DEGRAU_ACELERAR 150                     //Limite máximo de aumento da força lógica por ciclo para as bordas
#define DEGRAU_FREAR 35                         //Limite máximo de aumento da força lógica por ciclo para o centro

#define MAX_JOYSTICK 512
#define MAX_PWM 1023
#define TEMPO_DELAY 20 

// Variáveis globais
int left_x, left_y, right_x, right_y;
// Isso aqui só mantive do exemplo
// Custom "instance"
typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;  // which "seat" is being used
} my_platform_instance_t;

// Declarations (Só mantive do exemplo)
static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

// Variáveis globais


// iniciar motores "desligados" 
void InitGpioMotores(){
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) | (1ULL << IN3_PIN) | (1ULL << IN4_PIN),// avisa pra esp32 que aquele pino ta ligado colocando o bit 1 na posição especifica
        .mode = GPIO_MODE_OUTPUT,  // define pino como saida
        .pull_up_en = GPIO_PULLUP_DISABLE, // nao precisa de pull up, pois o ledc ja vai cuidar disso
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // nao precisa de pull down, pois o ledc ja vai cuidar disso
        .intr_type = GPIO_INTR_DISABLE //nao precisa pq o motor nao vai precisar interromper o codigo
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(IN1_PIN,0);
    gpio_set_level(IN2_PIN,0);
    gpio_set_level(IN3_PIN,0);  
    gpio_set_level(IN4_PIN,0);
}

void InitPwmMotores() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_a = {
        .speed_mode     = LEDC_MODE,
        .channel        = CHANNEL_A,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ENA_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_a));

    ledc_channel_config_t ledc_channel_b = {
        .speed_mode     = LEDC_MODE,
        .channel        = CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ENB_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));
}

int Deadzone(int valor_analogico) {
     // Valores entre -DEADZONE e DEADZONE serão considerados zero
    if(abs(valor_analogico) < DEADZONE) {
        return 0;
    }
    return valor_analogico;
}

void ProcessarMovimento(int x, int y) {

    // Variáveis estáticas para lembrar posição do analogico no ciclo anterior
    static int atualEsq = 0;
    static int atualDir = 0;
    // Variáveis estáticas para lembrar a direção no ciclo anterior (considerando 1 = Frente, 0 = Trás)
    static int direcaoAntigaEsq = 1; 
    static int direcaoAntigaDir = 1;


    int alvoEsq = y + x;
    int alvoDir = y - x;

    // Limita o alvo entre -512 e +512 (trava de segurança)
    if(alvoEsq > MAX_JOYSTICK){
        alvoEsq = MAX_JOYSTICK;
    }
    if(alvoEsq < -MAX_JOYSTICK){  
        alvoEsq = -MAX_JOYSTICK;
    }
    if(alvoDir > MAX_JOYSTICK){
        alvoDir = MAX_JOYSTICK;
    }
    if(alvoDir < -MAX_JOYSTICK){
        alvoDir = -MAX_JOYSTICK;
    } 

    // se o analogico estiver indo em direção ao centro o passo maximo é 35
    // mas se o analogico estiver indo para fora o passo maximo é 150
    int passoEsq;
    if(abs(alvoEsq) > abs(atualEsq) && (alvoEsq * atualEsq >= 0)) {
        passoEsq = DEGRAU_ACELERAR;
    }else{
        passoEsq = DEGRAU_FREAR; 
    }

    // Aplica o passo para chegar no alvo
    if(alvoEsq > atualEsq + passoEsq){
        atualEsq += passoEsq;
    }else if (alvoEsq < atualEsq - passoEsq){
        atualEsq -= passoEsq;
    }else{
        atualEsq = alvoEsq;
    }

    int passoDir;
    if(abs(alvoDir) > abs(atualDir) && (alvoDir * atualDir >= 0)) {
        passoDir = DEGRAU_ACELERAR;
    }else{
        passoDir = DEGRAU_FREAR;
    }

    if(alvoDir > atualDir + passoDir){
        atualDir += passoDir;
    }else if (alvoDir < atualDir - passoDir){
        atualDir -= passoDir;
    }else{   
        atualDir = alvoDir;
    }
    
    // tranformar o valor para PWM 
    uint32_t dutyEsq = abs(atualEsq) * 2;
    if (dutyEsq > MAX_PWM){
        dutyEsq = MAX_PWM;
    }
    
    uint32_t dutyDir = abs(atualDir) * 2;
    if (dutyDir > MAX_PWM){
        dutyDir = MAX_PWM;
    }

    // Descobre qual é a direção atual baseada no sinal da força 
    int direcaoNovaEsq;
    if (atualEsq >= 0) {
        direcaoNovaEsq = 1;
    } else {
        direcaoNovaEsq = 0;
    }

    int direcaoNovaDir;
    if (atualDir >= 0) {
        direcaoNovaDir = 1;
    } else {
        direcaoNovaDir = 0;
    }

    // Se a direção de algum dos motores mudar de um delay de 20ms e zere o PWM
    if (direcaoNovaEsq != direcaoAntigaEsq || direcaoNovaDir != direcaoAntigaDir) {
        
        // zera o pwm
        ledc_set_duty(LEDC_MODE, CHANNEL_A, 0);
        ledc_update_duty(LEDC_MODE, CHANNEL_A);
        ledc_set_duty(LEDC_MODE, CHANNEL_B, 0);
        ledc_update_duty(LEDC_MODE, CHANNEL_B);
        
        // aplica 20ms de delay
        vTaskDelay(pdMS_TO_TICKS(TEMPO_DELAY));
        
        //atualiza pro proximo ciclo
        direcaoAntigaEsq = direcaoNovaEsq;
        direcaoAntigaDir = direcaoNovaDir;
    }
    
    // Configura o Motor Esquerdo
    
    if (atualEsq >= 0) { 
        // Indo para Frente ou Parado
        gpio_set_level(IN1_PIN, 1); 
        gpio_set_level(IN2_PIN, 0);
    } else { 
        // Indo para Trás
        gpio_set_level(IN1_PIN, 0); 
        gpio_set_level(IN2_PIN, 1);
    }
    ledc_set_duty(LEDC_MODE, CHANNEL_A,dutyEsq);
    ledc_update_duty(LEDC_MODE, CHANNEL_A);

    // Configura o Motor Direito
    if (atualDir >= 0) {
        // Indo para Frente ou Parado
        gpio_set_level(IN3_PIN, 1);
        gpio_set_level(IN4_PIN, 0);
    } else {
        // Indo para Trás
        gpio_set_level(IN3_PIN, 0); 
        gpio_set_level(IN4_PIN, 1);
    }
    ledc_set_duty(LEDC_MODE, CHANNEL_B, dutyDir);
    ledc_update_duty(LEDC_MODE, CHANNEL_B);
}



// É nessa função que vão as configurações, FreeRTOS e afins
static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    logi("Bluepad32: Inicializando configuração\n");

    InitGpioMotores(); // Liga e configura as GPIOs de direção
    InitPwmMotores();  // Liga e configura os canais de velocidade (PWM)
    
    logi("Motores e PWM configurados com sucesso!\n");
    
}

static void my_platform_on_init_complete(void) {
    logi("Bluepad32: Configuração concluída\n");

    // Inicia a busca por dispositivos
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);

    // Baseada no tempo de execução, isso deleta ou lista as    chaves Bluetooth armazenadas
    if (1)
        uni_bt_del_keys_unsafe();
    else
        uni_bt_list_keys_unsafe();
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    // You can filter discovered devices here.
    // Just return any value different from UNI_ERROR_SUCCESS;
    // @param addr: the Bluetooth address
    // @param name: could be NULL, could be zero-length, or might contain the name.
    // @param cod: Class of Device. See "uni_bt_defines.h" for possible values.
    // @param rssi: Received Signal Strength Indicator (RSSI) measured in dBms. The higher (255) the better.

    // As an example, if you want to filter out keyboards, do:
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        logi("Ignoring keyboard\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logi("Bluepad32: Dispositivo conectado: %p\n", d);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("Bluepad32: Dispositivo desconectado: %p\n", d);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logi("Bluepad32: Dispositivo pronto: %p\n", d);
    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    trigger_event_on_gamepad(d);
    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uni_controller_t prev = {0};
    
    // Variável para armazenar as informações recebidas do controle
    // Basicamente vai em:
    // componentes/bluepad32/include/controller/uni_gamepad.h
    // Lá tem todas as structs de valores que você pode receber
    uni_gamepad_t* gp;
    uni_gamepad_mappings_t* tg;

    // Comenta isso aqui no projeto final, no exemplo diz que é para não ter
    // spam desnecessário de logs
    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }
    prev = *ctl;

    // Essa função é para garantir que só vai executar ações caso
    // o que tenha se conectado seja um controle do tipo GAMEPAD
    // no caso o controle de PS4, se for testar algo diferente, coemnta o if
    if(ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }

    // Mudando a cor do led do controle para verde (8 bits)
    //                                     r,  g,  0
    d->report_parser.set_lightbar_color(d, 0, 255, 0);
    
    // Extrai os dados do controle
    gp = &ctl->gamepad;

    // Valores dos analógicos
    left_x = gp->axis_x;
    left_y = gp->axis_y;
    right_x = gp->axis_rx;
    right_y = gp->axis_ry;

    //aplicando Deadzone
    left_x = Deadzone(left_x);
    left_y = Deadzone(left_y);

    ProcessarMovimento(left_x, left_y);
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
            uni_hid_device_t* d = data;

            if (d == NULL) {
                loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
                return;
            }
            logi("custom: on_device_oob_event(): %d\n", event);

            my_platform_instance_t* ins = get_my_platform_instance(d);
            ins->gamepad_seat = ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

            trigger_event_on_gamepad(d);
            break;
        }

        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            logi("custom: Bluetooth enabled: %d\n", (bool)(data));
            break;

        default:
            logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
            break;
    }
}

//
// Helpers
//
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);

    if (d->report_parser.play_dual_rumble != NULL) {
        d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 150 /* duration ms */, 128 /* weak magnitude */,
                                          40 /* strong magnitude */);
    }

    if (d->report_parser.set_player_leds != NULL) {
        d->report_parser.set_player_leds(d, ins->gamepad_seat);
    }

    if (d->report_parser.set_lightbar_color != NULL) {
        uint8_t red = (ins->gamepad_seat & 0x01) ? 0xff : 0;
        uint8_t green = (ins->gamepad_seat & 0x02) ? 0xff : 0;
        uint8_t blue = (ins->gamepad_seat & 0x04) ? 0xff : 0;
        d->report_parser.set_lightbar_color(d, red, green, blue);
    }
}

//
// Entry Point
//
struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "custom",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };

    return &plat;
}
