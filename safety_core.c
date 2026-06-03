/*============================================================================
 *testy logiki na PC (gcc). Nie trafia do STM32.
 *==========================================================================*/
#include <stdio.h>
#include <string.h>
#include "safety_core.h"

static int fails = 0;
#define CHECK(cond, msg) do{ if(!(cond)){ printf("  FAIL: %s\n", msg); fails++; } }while(0)

/* domyslna, rozsadna konfiguracja testowa */
static safety_config_t cfg = {
    .i_max_ma = 50, .v_max_mv = 5000, .temp_max_c = 42,
    .i_sensor_min_raw = 5, .i_sensor_max_raw = 4090,
    .max_on_ms = 1000, .max_session_ms = 3000, .cooldown_ms = 2000,
    .heartbeat_timeout_ms = 50, .amp_max_setpoint = 2048
};

/* "zdrowe" wejscia bazowe */
static safety_inputs_t healthy(uint32_t t){
    safety_inputs_t in; memset(&in,0,sizeof in);
    in.estop_active=false; in.fpga_heartbeat_ok=true; in.supply_ok=true;
    in.out_current_ma=10; in.out_voltage_mv=2000; in.temperature_c=30;
    in.i_sensor_raw=2000; in.commanded_setpoint=1000; in.now_ms=t;
    return in;
}

/* wywolanie safety_step ze "zdrowymi" wejsciami w czasie t (adres lokalnej) */
#define STEP_H(ctxp, t) ({ safety_inputs_t _in = healthy(t); \
                           safety_step((ctxp), &cfg, &_in); })

int main(void){
    safety_ctx_t ctx; safety_outputs_t o;

    /* T1: boot INIT->IDLE, wyjscie wylaczone */
    safety_init(&ctx);
    o = safety_step(&ctx,&cfg,&(safety_inputs_t){.now_ms=0,.fpga_heartbeat_ok=true,.supply_ok=true,.i_sensor_raw=2000,.temperature_c=30});
    CHECK(o.state==SS_IDLE, "boot -> IDLE");
    CHECK(o.output_enable==false, "boot: wyjscie OFF");

    /* T2: start -> ARMING -> ACTIVE, wyjscie ON, amplituda OK */
    { safety_inputs_t in=healthy(10); in.request_start=true;
      o=safety_step(&ctx,&cfg,&in); CHECK(o.state==SS_ARMING,"->ARMING"); }
    { safety_inputs_t in=healthy(20);
      o=safety_step(&ctx,&cfg,&in); CHECK(o.state==SS_ACTIVE,"->ACTIVE"); }
    { safety_inputs_t in=healthy(30);
      o=safety_step(&ctx,&cfg,&in);
      CHECK(o.output_enable==true,"ACTIVE: wyjscie ON");
      CHECK(o.dac_setpoint==1000,"ACTIVE: setpoint przekazany"); }

    /* T3: amplituda powyzej limitu -> przyciecie */
    { safety_inputs_t in=healthy(40); in.commanded_setpoint=9999;
      o=safety_step(&ctx,&cfg,&in);
      CHECK(o.dac_setpoint==cfg.amp_max_setpoint,"amplituda przycieta do max"); }

    /* T4: nadprad w ACTIVE -> FAULT, zatrzask mimo powrotu do normy */
    { safety_inputs_t in=healthy(50); in.out_current_ma=80;
      o=safety_step(&ctx,&cfg,&in);
      CHECK(o.state==SS_FAULT,"nadprad -> FAULT");
      CHECK(o.fault==FAULT_OVERCURRENT,"kod = OVERCURRENT");
      CHECK(o.output_enable==false,"FAULT: wyjscie OFF"); }
    { safety_inputs_t in=healthy(60); /* prad juz normalny */
      o=safety_step(&ctx,&cfg,&in);
      CHECK(o.state==SS_FAULT,"FAULT zatrzasniety");
      CHECK(o.output_enable==false,"nadal OFF"); }

    /* T5: kasowanie bledu dopiero gdy warunek ustal */
    { safety_inputs_t in=healthy(70); in.request_fault_reset=true;
      o=safety_step(&ctx,&cfg,&in);
      CHECK(o.state==SS_INIT||o.state==SS_IDLE,"reset -> wyjscie z FAULT"); }

    /* T6: E-STOP natychmiast -> FAULT z dowolnego stanu */
    safety_init(&ctx);
    STEP_H(&ctx,0); /* -> IDLE */
    { safety_inputs_t in=healthy(5); in.estop_active=true;
      o=safety_step(&ctx,&cfg,&in);
      CHECK(o.state==SS_FAULT && o.fault==FAULT_ESTOP,"E-STOP -> FAULT"); }

    /* T7: utrata tetna FPGA w ACTIVE -> FAULT */
    safety_init(&ctx);
    STEP_H(&ctx,0);
    { safety_inputs_t in=healthy(10); in.request_start=true; safety_step(&ctx,&cfg,&in); }
    STEP_H(&ctx,20); /* ACTIVE */
    { safety_inputs_t in=healthy(30); in.fpga_heartbeat_ok=false;
      o=safety_step(&ctx,&cfg,&in);
      CHECK(o.state==SS_FAULT && o.fault==FAULT_FPGA_HEARTBEAT,"brak tetna -> FAULT"); }

    /* T8: limit czasu ciaglego -> COOLDOWN (OFF), potem IDLE */
    safety_init(&ctx);
    STEP_H(&ctx,0);
    { safety_inputs_t in=healthy(10); in.request_start=true; safety_step(&ctx,&cfg,&in); }
    STEP_H(&ctx,20);
    o=STEP_H(&ctx,1100); /* on_ms=1080 > max_on=1000 */
    CHECK(o.state==SS_COOLDOWN,"limit ciagly -> COOLDOWN");
    CHECK(o.output_enable==false,"COOLDOWN: wyjscie OFF");
    o=STEP_H(&ctx,1100+2000+1); /* po cooldown */
    CHECK(o.state==SS_IDLE,"po odpoczynku -> IDLE");

    /* T9: wyjscie NIGDY wlaczone poza ACTIVE (skan stanow) */
    {
        safety_state_e bad_on=SS_INIT; int ok=1;
        safety_init(&ctx);
        for(uint32_t t=0;t<50;t++){
            safety_inputs_t in=healthy(t*10);
            if(t==2) in.request_start=true;
            if(t==10) in.estop_active=true; /* wymus FAULT po drodze */
            o=safety_step(&ctx,&cfg,&in);
            if(o.output_enable && o.state!=SS_ACTIVE){ ok=0; bad_on=o.state; }
        }
        CHECK(ok, "wyjscie ON wylacznie w stanie ACTIVE");
        if(!ok) printf("    (wlaczone w stanie %d)\n", bad_on);
    }

    if(fails==0) printf("=== WSZYSTKIE TESTY OK ===\n");
    else         printf("=== BLEDY: %d ===\n", fails);
    return fails ? 1 : 0;
}