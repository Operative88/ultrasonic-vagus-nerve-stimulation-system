/*============================================================================
 *  safety_core.h
 *----------------------------------------------------------------------------
 *  Niezalezny od sprzetu rdzen logiki bezpieczenstwa dla ukladu
 *  nieinwazyjnej stymulacji nerwu blednego (FPGA + DAC + wzmacniacz + STM32).
 *
 *  ZASADA: ta warstwa NIE dotyka rejestrow STM32. Operuje wylacznie na
 *  liczbach (zmierzone wartosci) i zwraca decyzje. Dzieki temu da sie ja
 *  w calosci przetestowac na PC (patrz test_safety_core.c) - a wlasnie
 *  logika bezpieczenstwa jest miejscem, gdzie blad jest najgrozniejszy.
 *
 *  STM32 pelni tu role NIEZALEZNEGO NADZORCY (safety supervisor): druga
 *  para oczu nad FPGA/DAC/wzmacniaczem, z wlasnym, odrebnym licznikiem dawki.
 *==========================================================================*/
#ifndef SAFETY_CORE_H
#define SAFETY_CORE_H

#include <stdint.h>
#include <stdbool.h>

/* --- Stany maszyny bezpieczenstwa ------------------------------------- */
typedef enum {
    SS_INIT = 0,   /* po starcie: wszystko wylaczone, czeka na walidacje  */
    SS_IDLE,       /* gotowy, wyjscie wylaczone                           */
    SS_ARMING,     /* operator zazadal startu: ponowna walidacja          */
    SS_ACTIVE,     /* stymulacja dozwolona (jedyny stan z wyjsciem ON)    */
    SS_COOLDOWN,   /* obowiazkowy odpoczynek po limicie dawki             */
    SS_FAULT       /* blad zatrzasniety: wyjscie OFF do czasu kasowania   */
} safety_state_e;

/* --- Kody bledow (zatrzaskiwany jest PIERWSZY) ------------------------ */
typedef enum {
    FAULT_NONE = 0,
    FAULT_ESTOP,           /* wcisniety przycisk awaryjny                 */
    FAULT_FPGA_HEARTBEAT,  /* brak "tetna" z FPGA (FPGA zawieszony)       */
    FAULT_SUPPLY,          /* napiecie zasilania poza zakresem            */
    FAULT_OVERCURRENT,     /* przekroczony prad wyjsciowy                 */
    FAULT_OVERVOLTAGE,     /* przekroczone napiecie wyjsciowe             */
    FAULT_OVERTEMP,        /* przegrzanie przetwornika/wzmacniacza        */
    FAULT_SENSOR_RANGE     /* czujnik poza zakresem (rozwarcie/zwarcie)   */
} fault_code_e;

/* --- Konfiguracja (progi i limity) ------------------------------------ */
typedef struct {
    uint16_t i_max_ma;            /* prog nadpradowy                      */
    uint16_t v_max_mv;            /* prog nadnapieciowy                   */
    int16_t  temp_max_c;          /* prog przegrzania                     */
    uint16_t i_sensor_min_raw;    /* dolna granica wiarygodnosci ADC pradu*/
    uint16_t i_sensor_max_raw;    /* gorna granica wiarygodnosci ADC pradu*/
    uint32_t max_on_ms;           /* maks. czas CIAGLEJ stymulacji        */
    uint32_t max_session_ms;      /* maks. czas SUMARYCZNY w sesji        */
    uint32_t cooldown_ms;         /* obowiazkowy odpoczynek               */
    uint32_t heartbeat_timeout_ms;/* maks. odstep miedzy tetnami FPGA     */
    uint16_t amp_max_setpoint;    /* twardy limit amplitudy (DAC)         */
} safety_config_t;

/* --- Wejscia (juz przeliczone wartosci fizyczne + flagi) -------------- */
typedef struct {
    bool     estop_active;        /* true = awaryjny wcisniety            */
    bool     fpga_heartbeat_ok;   /* true = tetno FPGA swieze             */
    bool     supply_ok;           /* true = zasilanie w normie            */
    uint16_t out_current_ma;
    uint16_t out_voltage_mv;
    int16_t  temperature_c;
    uint16_t i_sensor_raw;        /* surowy ADC pradu (kontrola czujnika) */
    uint16_t commanded_setpoint;  /* zadana amplituda (do przyciecia)     */
    bool     request_start;
    bool     request_stop;
    bool     request_fault_reset;
    uint32_t now_ms;              /* czas monotoniczny [ms]               */
} safety_inputs_t;

/* --- Kontekst (stan trwaly miedzy krokami) ---------------------------- */
typedef struct {
    safety_state_e state;
    fault_code_e   fault;
    uint32_t       on_since_ms;
    uint32_t       session_accum_ms;
    uint32_t       cooldown_until_ms;
    uint32_t       last_now_ms;
    bool           first_step;
} safety_ctx_t;

/* --- Wyjscia (decyzje dla warstwy sprzetowej) ------------------------- */
typedef struct {
    bool           output_enable; /* steruje glownym ryglem / EN wzmacniacza */
    bool           dac_allow;     /* zezwolenie na sygnal DAC              */
    uint16_t       dac_setpoint;  /* PRZYCIETA amplituda do podania        */
    safety_state_e state;
    fault_code_e   fault;
} safety_outputs_t;

/* --- API -------------------------------------------------------------- */
void             safety_init(safety_ctx_t *ctx);
safety_outputs_t safety_step(safety_ctx_t *ctx,
                             const safety_config_t *cfg,
                             const safety_inputs_t *in);

#endif /* SAFETY_CORE_H */