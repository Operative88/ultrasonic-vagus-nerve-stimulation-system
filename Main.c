
#include "stm32f4xx_hal.h"
#include "safety_core.h"

/* uchwyty z CubeMX */
ADC_HandleTypeDef  hadc1;
IWDG_HandleTypeDef hiwdg;
DMA_HandleTypeDef  hdma_adc1;

/* zadana amplituda (np. ustawiana z UART/UI); domyslnie bezpiecznie 0 */
volatile uint16_t g_commanded_setpoint = 0;

/* deklaracje warstwy HAL */
void safety_hal_init(uint32_t heartbeat_timeout_ms);
void safety_hal_sample(safety_inputs_t *in, uint32_t now_ms);
void safety_hal_apply(const safety_outputs_t *out);

/* prototypy z CubeMX */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);

/* ----------------------- LIMITY BEZPIECZENSTWA -------------------------- */
/* Wartosci PRZYKLADOWE - dobierz konserwatywnie do swojego sprzetu i
 * protokolu, najlepiej z marginesem ponizej granic bezpieczenstwa tkanki. */
static const safety_config_t cfg = {
    .i_max_ma             = 50,       /* prog nadpradowy [mA]              */
    .v_max_mv             = 5000,     /* prog nadnapieciowy [mV]           */
    .temp_max_c           = 42,       /* prog przegrzania [°C]             */
    .i_sensor_min_raw     = 5,        /* czujnik pradu: ponizej => awaria  */
    .i_sensor_max_raw     = 4090,     /* czujnik pradu: powyzej => awaria  */
    .max_on_ms            = 30000,    /* maks. 30 s ciaglej stymulacji     */
    .max_session_ms       = 300000,   /* maks. 5 min sumarycznie / sesje   */
    .cooldown_ms          = 60000,    /* obowiazkowy odpoczynek 60 s       */
    .heartbeat_timeout_ms = 50,       /* brak tetna FPGA > 50 ms => FAULT  */
    .amp_max_setpoint     = 2048      /* twardy limit amplitudy DAC        */
};

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_IWDG_Init();

    safety_ctx_t ctx;
    safety_init(&ctx);
    safety_hal_init(cfg.heartbeat_timeout_ms);

    uint32_t last_tick = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if (now != last_tick)            /* kadencja 1 ms */
        {
            last_tick = now;

            safety_inputs_t in;
            safety_hal_sample(&in, now);

            safety_outputs_t out = safety_step(&ctx, &cfg, &in);

            safety_hal_apply(&out);

            /* Watchdog odswiezamy WYLACZNIE po wykonaniu pelnego kroku
             * nadzorcy. Jesli petla/krok sie zawiesi -> brak odswiezenia
             * -> reset MCU -> SAFETY_EN w stan niski (fail-safe). */
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}