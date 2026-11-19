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

/* ─── Pinos de direção (L298N) ─────────────────────────────────── */
#define IN1_PIN  GPIO_NUM_32    // Motor Esquerdo (A) - Frente
#define IN2_PIN  GPIO_NUM_33    // Motor Esquerdo (A) - Trás
#define IN3_PIN  GPIO_NUM_25    // Motor Direito (B)  - Frente
#define IN4_PIN  GPIO_NUM_26    // Motor Direito (B)  - Trás

/* ─── Pinos de velocidade (PWM) ────────────────────────────────── */
#define ENA_PIN  GPIO_NUM_19    // Enable Motor Esquerdo (A)
#define ENB_PIN  GPIO_NUM_18    // Enable Motor Direito (B)

/* ─── Configuração LEDC PWM ────────────────────────────────────── */
#define PWM_FREQ_HZ     4000                    // 4 kHz (ideal para L298N, evita perda de torque em alta freq)
#define PWM_RESOLUTION  LEDC_TIMER_10_BIT       // 1024 níveis (0–1023)
#define MAX_PWM         1023                    // (2^10 - 1)

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE

#define CHANNEL_A       LEDC_CHANNEL_0          // Canal PWM para Motor A
#define CHANNEL_B       LEDC_CHANNEL_1          // Canal PWM para Motor B

/* ─── Parâmetros de Controle e Dirigibilidade ──────────────────── */
#define DEADZONE        40      // Zona morta do analógico (mesmo padrão do BTS7960)
#define MAX_JOYSTICK    512.0f  // Valor máximo do eixo no Bluepad32
#define FATOR_CURVA     2.0f    // Atenuação da curva (1.0 = giro agressivo no próprio eixo, 2.0 a 5.0 = curva mais suave)

// Rampa de aceleração/frenagem (executada a cada 20ms na Task)
#define DEGRAU_ACELERAR 150     // Passo máximo por ciclo (20ms) ao acelerar
#define DEGRAU_FREAR    80      // Passo máximo por ciclo (20ms) ao desacelerar/retornar ao centro
#define TEMPO_DELAY_REV 15      // Delay (ms) de proteção ao inverter sentido de rotação

// Variáveis globais (volatile pois são escritas no callback BT e lidas na Task)
volatile int left_x = 0, left_y = 0, right_x = 0, right_y = 0;

// Custom "instance"
typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;
} my_platform_instance_t;

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

// Inicializa os pinos digitais da Ponte H L298N
void InitGpioMotores(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) | (1ULL << IN3_PIN) | (1ULL << IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(IN1_PIN, 0);
    gpio_set_level(IN2_PIN, 0);
    gpio_set_level(IN3_PIN, 0);
    gpio_set_level(IN4_PIN, 0);
}

// Inicializa os canais de PWM no periférico LEDC
void InitPwmMotores(void) {
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

// Função de Deadzone para limpar ruído do analógico
int apply_deadzone(int value, int deadzone) {
    if (abs(value) < deadzone) {
        return 0;
    }
    return value;
}

// Função auxiliar para aplicar a rampa de aceleração/frenagem por motor
int aplicar_rampa(int alvo, int atual) {
    int passo;
    // Se estiver acelerando na mesma direção, usa DEGRAU_ACELERAR; caso contrário, usa DEGRAU_FREAR
    if (abs(alvo) > abs(atual) && (alvo * atual >= 0)) {
        passo = DEGRAU_ACELERAR;
    } else {
        passo = DEGRAU_FREAR;
    }

    if (alvo > atual + passo) {
        return atual + passo;
    } else if (alvo < atual - passo) {
        return atual - passo;
    } else {
        return alvo;
    }
}

// Task dedicada para leitura contínua dos eixos e atualização da Ponte H L298N
void TaskControleMotoresL298N(void *pvParameter) {
    InitGpioMotores();
    InitPwmMotores();

    int atualEsq = 0;
    int atualDir = 0;

    // 1 = Frente, -1 = Ré, 0 = Parado
    int dirAntigaEsq = 0;
    int dirAntigaDir = 0;

    logi("Task L298N inicializada com sucesso!\n");

    while (1) {
        // 1. Leitura com Deadzone
        // OBS: No Bluepad32 (PS4), empurrar o analógico para FRENTE gera left_y NEGATIVO (-512).
        // Por isso invertemos o sinal de left_y (-left_y) para que FRENTE seja POSITIVO (+512)
        // e TRÁS seja NEGATIVO (-512), facilitando toda a matemática de mistura.
        float x_axis = (float)apply_deadzone(left_x, DEADZONE);
        float y_axis = (float)apply_deadzone(-left_y, DEADZONE);

        // 2. Cálculo da curva (seguindo a lógica do BTS7960)
        float curva = x_axis / FATOR_CURVA;

        // Se o robô estiver parado no eixo Y e você mexer só o X, permite girar no próprio eixo
        // Se quiser igualzinho ao BTS7960 (mesmo fator sempre), basta manter a soma direta:
        int alvoEsq = (int)(y_axis + curva);
        int alvoDir = (int)(y_axis - curva);

        // 3. Limita os valores máximos e mínimos (-512 a +512)
        if (alvoEsq > (int)MAX_JOYSTICK)  alvoEsq = (int)MAX_JOYSTICK;
        if (alvoEsq < -(int)MAX_JOYSTICK) alvoEsq = -(int)MAX_JOYSTICK;
        if (alvoDir > (int)MAX_JOYSTICK)  alvoDir = (int)MAX_JOYSTICK;
        if (alvoDir < -(int)MAX_JOYSTICK) alvoDir = -(int)MAX_JOYSTICK;

        // 4. Aplica a rampa de aceleração/frenagem a cada 20ms
        atualEsq = aplicar_rampa(alvoEsq, atualEsq);
        atualDir = aplicar_rampa(alvoDir, atualDir);

        // 5. Determina a direção atual de cada motor (1 = Frente, -1 = Ré, 0 = Parado)
        int dirNovaEsq = (atualEsq > 0) ? 1 : ((atualEsq < 0) ? -1 : 0);
        int dirNovaDir = (atualDir > 0) ? 1 : ((atualDir < 0) ? -1 : 0);

        // 6. Proteção contra curto/pico de corrente na reversão brusca de sentido
        bool inverteuEsq = (dirNovaEsq != 0 && dirAntigaEsq != 0 && dirNovaEsq != dirAntigaEsq);
        bool inverteuDir = (dirNovaDir != 0 && dirAntigaDir != 0 && dirNovaDir != dirAntigaDir);

        if (inverteuEsq || inverteuDir) {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, CHANNEL_A, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, CHANNEL_A));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, CHANNEL_B, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, CHANNEL_B));

            gpio_set_level(IN1_PIN, 0);
            gpio_set_level(IN2_PIN, 0);
            gpio_set_level(IN3_PIN, 0);
            gpio_set_level(IN4_PIN, 0);

            vTaskDelay(pdMS_TO_TICKS(TEMPO_DELAY_REV));
        }

        if (dirNovaEsq != 0) dirAntigaEsq = dirNovaEsq;
        if (dirNovaDir != 0) dirAntigaDir = dirNovaDir;

        // 7. Cálculo do Duty Cycle do PWM (0 a 1023)
        uint32_t dutyEsq = (uint32_t)((fabsf((float)atualEsq) / MAX_JOYSTICK) * MAX_PWM);
        uint32_t dutyDir = (uint32_t)((fabsf((float)atualDir) / MAX_JOYSTICK) * MAX_PWM);

        if (dutyEsq > MAX_PWM) dutyEsq = MAX_PWM;
        if (dutyDir > MAX_PWM) dutyDir = MAX_PWM;

        // 8. Acionamento dos pinos de direção e PWM - MOTOR ESQUERDO (A)
        if (atualEsq > 0) {
            gpio_set_level(IN1_PIN, 1);
            gpio_set_level(IN2_PIN, 0);
        } else if (atualEsq < 0) {
            gpio_set_level(IN1_PIN, 0);
            gpio_set_level(IN2_PIN, 1);
        } else {
            // Ponto morto (Coast) quando o controle está solto
            gpio_set_level(IN1_PIN, 0);
            gpio_set_level(IN2_PIN, 0);
        }
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, CHANNEL_A, dutyEsq));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, CHANNEL_A));

        // 9. Acionamento dos pinos de direção e PWM - MOTOR DIREITO (B)
        if (atualDir > 0) {
            gpio_set_level(IN3_PIN, 1);
            gpio_set_level(IN4_PIN, 0);
        } else if (atualDir < 0) {
            gpio_set_level(IN3_PIN, 0);
            gpio_set_level(IN4_PIN, 1);
        } else {
            // Ponto morto (Coast) quando o controle está solto
            gpio_set_level(IN3_PIN, 0);
            gpio_set_level(IN4_PIN, 0);
        }
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, CHANNEL_B, dutyDir));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, CHANNEL_B));

        // Ciclo fixo de 20ms (50Hz) igual ao código do BTS7960
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Inicialização da plataforma e criação da Task do FreeRTOS
static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    logi("Bluepad32: Inicializando configuração para L298N\n");

    // Cria a Task que gerencia o PWM e os pinos do L298N continuamente
    xTaskCreate(&TaskControleMotoresL298N, "Controle L298N", 4096, NULL, 1, NULL);
}

static void my_platform_on_init_complete(void) {
    logi("Bluepad32: Configuração concluída\n");

    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);

    if (1)
        uni_bt_del_keys_unsafe();
    else
        uni_bt_list_keys_unsafe();
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
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
    // Segurança: para o robô imediatamente se o controle de PS4 desconectar
    left_x = 0;
    left_y = 0;
    right_x = 0;
    right_y = 0;
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
    uni_gamepad_t* gp;

    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }
    prev = *ctl;

    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }

    // Muda a cor do LED do controle para verde
    if (d->report_parser.set_lightbar_color != NULL) {
        d->report_parser.set_lightbar_color(d, 0, 255, 0);
    }

    gp = &ctl->gamepad;

    // Apenas atualiza as variáveis globais (sem bloquear o callback Bluetooth)
    left_x = gp->axis_x;
    left_y = gp->axis_y;
    right_x = gp->axis_rx;
    right_y = gp->axis_ry;
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
        d->report_parser.play_dual_rumble(d, 0, 150, 128, 40);
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