/*============================================================================
 *  stm32_safety_hal.c
 *----------------------------------------------------------------------------
 *  Warstwa sprzetowa (STM32F4 HAL) laczaca przenosny rdzen safety_core
 *  z peryferiami STM32. Tu zyja przerwania, ADC i sterowanie ryglem.
 *
 *  ZALOZENIA SPRZETOWE 
 *    - SAFETY_EN  : wyjscie push-pull, steruje EN wzmacniacza / load-switch.
 *                   *** WYMAGANY zewnetrzny rezystor sciagajacy do masy ***,
 *                   aby przy resecie/braku zasilania MCU wzmacniacz byl OFF.
 *    - ESTOP      : wejscie z pull-up, przycisk zwiera do masy (wcisniety=0).
 *                   Powinien TEZ sprzetowo odcinac zasilanie wzmacniacza,
 *                   niezaleznie od MCU.
 *    - FPGA_HB    : wejscie, "tetno" z FPGA (np. LED7/heartbeat z projektu
 *                   FPGA). Zbocze narastajace generuje przerwanie.
 *    - ADC1 (DMA) : 3 kanaly -> [0]=prad wyj., [1]=napiecie wyj., [2]=NTC temp.
 *    - IWDG       : niezalezny watchdog (~50 ms).
 *    - PVD        : detekcja zaniku napiecia VDD (zasilanie OK?).
 *    - Przyciski  : START / STOP / RESET (z pull-up, wcisniety=0).
 *==========================================================================*/
#include "stm32f4xx_hal.h"
#include "safety_core.h"

/* ----------------------- MAPOWANIE PINOW (do edycji) -------------------- */
#define SAFETY_EN_PORT   GPIOA
#define SAFETY_EN_PIN    GPIO_PIN_5
#define DAC_ALLOW_PORT   GPIOA
#define DAC_ALLOW_PIN    GPIO_PIN_6
#define FAULT_LED_PORT   GPIOA
#define FAULT_LED_PIN    GPIO_PIN_7

#define ESTOP_PIN        GPIO_PIN_0   /* EXTI0 */
#define FPGA_HB_PIN      GPIO_PIN_1   /* EXTI1 */
#define BTN_START_PORT   GPIOB
#define BTN_START_PIN    GPIO_PIN_4
#define BTN_STOP_PORT    GPIOB
#define BTN_STOP_PIN     GPIO_PIN_5
#define BTN_RESET_PORT   GPIOB
#define BTN_RESET_PIN    GPIO_PIN_6

/* SKALOWANIE ADC  */
/* Przyklad dla 12-bit ADC, Vref=3.3 V. Wartosci zalezne od Twoich
 * dzielnikow/czujnikow - MUSISZ je wyznaczyc pomiarem. */
#define ADC_FULL_SCALE   4095u
#define VREF_MV          3300u
#define I_SENSE_MA_PER_LSB_NUM  1   /* prad[mA] = raw * NUM / DEN  (przyklad) */
#define I_SENSE_MA_PER_LSB_DEN  10
#define V_DIVIDER_NUM    11         /* napiecie[mV] = mV_adc * NUM / DEN      */
#define V_DIVIDER_DEN    1
/* NTC: tu uproszczony model liniowy - w praktyce uzyj tablicy beta/Steinhart */
#define TEMP_OFFSET_C    (-40)
#define TEMP_MV_PER_C    10

/* ----------------------- UCHWYTY HAL (z CubeMX) ------------------------- */
extern ADC_HandleTypeDef  hadc1;
extern IWDG_HandleTypeDef hiwdg;

/* ----------------------- STAN WEWNETRZNY -------------------------------- */
static volatile uint16_t adc_dma[3];          /* [I, V, T] - wypelniane DMA */
static volatile uint32_t last_hb_ms   = 0;    /* czas ostatniego tetna FPGA */
static volatile bool     estop_latched = false;
static uint32_t          hb_timeout_ms = 50;  /* ustawiane w init           */

/* Natychmiastowe, sprzetowo-szybkie odciecie wyjscia (uzywane w ISR). */
static inline void force_output_off(void)
{
    HAL_GPIO_WritePin(SAFETY_EN_PORT, SAFETY_EN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DAC_ALLOW_PORT, DAC_ALLOW_PIN, GPIO_PIN_RESET);
}

/*===========================  INICJALIZACJA  ============================*/
void safety_hal_init(uint32_t heartbeat_timeout_ms)
{
    hb_timeout_ms = heartbeat_timeout_ms;

    /* wyjscie domyslnie BEZPIECZNE */
    force_output_off();

    /* ADC + DMA w trybie ciaglym do bufora 3 probek */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma, 3);

    /* detekcja zaniku VDD */
    PWR_PVDTypeDef pvd = {0};
    pvd.PVDLevel = PWR_PVDLEVEL_5;          /* ~2.8 V - dobierz do projektu */
    pvd.Mode     = PWR_PVD_MODE_IT_RISING_FALLING;
    HAL_PWR_ConfigPVD(&pvd);
    HAL_PWR_EnablePVD();

    /* niezalezny watchdog - jesli petla glowna stanie, MCU sie zresetuje
     * i SAFETY_EN wroci do stanu niskiego (rezystor sciagajacy). */
    /* (hiwdg konfigurowany w CubeMX na ~50 ms; tu tylko start) */
    HAL_IWDG_Init(&hiwdg);

    last_hb_ms = HAL_GetTick();
}

/*============================  PRZERWANIA  ==============================*/
/* Wspolny callback EXTI z HAL. */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == ESTOP_PIN) {
        /* SCIEZKA SZYBKA: odetnij wyjscie natychmiast, nie czekaj na petle */
        estop_latched = true;
        force_output_off();
    } else if (pin == FPGA_HB_PIN) {
        last_hb_ms = HAL_GetTick();         /* swieze tetno z FPGA          */
    }
}

/* PVD: zanik/powrot napiecia VDD - obsluga przez flage czytana w sample(). */
volatile bool g_supply_ok = true;
void HAL_PWR_PVDCallback(void)
{
    /* PWR->CSR PVDO=1 gdy VDD ponizej progu */
    g_supply_ok = (PWR->CSR & PWR_CSR_PVDO) ? false : true;
    if (!g_supply_ok) force_output_off();
}

/*======================  KONWERSJE ADC -> FIZYKA  =======================*/
static uint16_t raw_to_ma(uint16_t raw)
{
    return (uint16_t)((uint32_t)raw * I_SENSE_MA_PER_LSB_NUM / I_SENSE_MA_PER_LSB_DEN);
}
static uint16_t raw_to_mv(uint16_t raw)
{
    uint32_t mv_adc = (uint32_t)raw * VREF_MV / ADC_FULL_SCALE;
    return (uint16_t)(mv_adc * V_DIVIDER_NUM / V_DIVIDER_DEN);
}
static int16_t raw_to_temp_c(uint16_t raw)
{
    uint32_t mv = (uint32_t)raw * VREF_MV / ADC_FULL_SCALE;
    return (int16_t)((int32_t)(mv / TEMP_MV_PER_C) + TEMP_OFFSET_C);
}

/*=====================  ODCZYT WEJSC DLA RDZENIA  =======================*/
void safety_hal_sample(safety_inputs_t *in, uint32_t now_ms)
{
    in->now_ms = now_ms;

    /* tetno FPGA: swieze, jesli ostatnie zbocze bylo niedawno (odporne na
     * zwarcie linii do stalego poziomu - brak zbocz => timeout => FAULT). */
    in->fpga_heartbeat_ok = ((now_ms - last_hb_ms) < hb_timeout_ms);

    in->estop_active = estop_latched ||
        (HAL_GPIO_ReadPin(GPIOA, ESTOP_PIN) == GPIO_PIN_RESET);

    in->supply_ok = g_supply_ok;

    in->i_sensor_raw   = adc_dma[0];
    in->out_current_ma = raw_to_ma(adc_dma[0]);
    in->out_voltage_mv = raw_to_mv(adc_dma[1]);
    in->temperature_c  = raw_to_temp_c(adc_dma[2]);

    /* zadana amplituda - tu np. z UI/UART; placeholder: */
    extern volatile uint16_t g_commanded_setpoint;
    in->commanded_setpoint = g_commanded_setpoint;

    /* przyciski (aktywne w stanie niskim) */
    in->request_start = (HAL_GPIO_ReadPin(BTN_START_PORT, BTN_START_PIN) == GPIO_PIN_RESET);
    in->request_stop  = (HAL_GPIO_ReadPin(BTN_STOP_PORT,  BTN_STOP_PIN)  == GPIO_PIN_RESET);
    in->request_fault_reset =
                        (HAL_GPIO_ReadPin(BTN_RESET_PORT, BTN_RESET_PIN) == GPIO_PIN_RESET);

    if (in->request_fault_reset) estop_latched = false; /* skasuj zatrzask ESTOP */
}

/*=====================  WYSTAWIENIE DECYZJI NA PINY  ====================*/
void safety_hal_apply(const safety_outputs_t *out)
{
    HAL_GPIO_WritePin(SAFETY_EN_PORT, SAFETY_EN_PIN,
                      out->output_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DAC_ALLOW_PORT, DAC_ALLOW_PIN,
                      out->dac_allow ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FAULT_LED_PORT, FAULT_LED_PIN,
                      (out->state == SS_FAULT) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* out->dac_setpoint mozna tu wystawic na wewnetrzny DAC STM32 jako
     * napiecie odniesienia/limit wzmocnienia, albo wyslac do FPGA. */
}