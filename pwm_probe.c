/* pwm_probe.c - bench diagnostic, NOT part of the Simulink build.
   Drives ONE driver channel at a fixed PWM count for a few seconds so you can measure which
   physical output terminals respond. Same pins and drive sequence as act2_wrapper.c.

   Build on the Pi:   gcc pwm_probe.c -o pwm_probe -lwiringPi
   Run (stop Simulink first):
       ./pwm_probe pump  149      pump channel (GPIO18/17/27, PWM on GPIO13) at 149 counts
       ./pwm_probe valve 192      valve channel (GPIO14/5/6, PWM on GPIO12) at 192 counts
   Counts are capped at 192 here, same as the model's hard limit. Output is off after 10 s or Ctrl+C. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <wiringPi.h>

#define PUMP_EN 18
#define PUMP_INA 17
#define PUMP_INB 27
#define PUMP_PWM 13
#define SOL_EN 14
#define SOL_INA 5
#define SOL_INB 6
#define SOL_PWM 12
#define CAP 192

static void all_off(void)
{
    digitalWrite(PUMP_EN, LOW); digitalWrite(PUMP_INA, LOW); digitalWrite(PUMP_INB, LOW); pwmWrite(PUMP_PWM, 0);
    digitalWrite(SOL_EN, LOW);  digitalWrite(SOL_INA, LOW);  digitalWrite(SOL_INB, LOW);  pwmWrite(SOL_PWM, 0);
}
static void on_sig(int s) { all_off(); _exit(1); (void)s; }

int main(int argc, char **argv)
{
    int en, ina, inb, pwm, counts;
    if (argc != 3) { fprintf(stderr, "usage: %s pump|valve counts\n", argv[0]); return 2; }
    counts = atoi(argv[2]);
    if (counts < 0) counts = 0;
    if (counts > CAP) counts = CAP;
    if (!strcmp(argv[1], "pump"))       { en = PUMP_EN; ina = PUMP_INA; inb = PUMP_INB; pwm = PUMP_PWM; }
    else if (!strcmp(argv[1], "valve")) { en = SOL_EN;  ina = SOL_INA;  inb = SOL_INB;  pwm = SOL_PWM; }
    else { fprintf(stderr, "first argument must be pump or valve\n"); return 2; }

    if (wiringPiSetupGpio() == -1) return 1;
    pinMode(PUMP_EN, OUTPUT); pinMode(PUMP_INA, OUTPUT); pinMode(PUMP_INB, OUTPUT); pinMode(PUMP_PWM, PWM_OUTPUT);
    pinMode(SOL_EN, OUTPUT);  pinMode(SOL_INA, OUTPUT);  pinMode(SOL_INB, OUTPUT);  pinMode(SOL_PWM, PWM_OUTPUT);
    all_off();
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);

    printf("%s channel: EN=GPIO%d INA=GPIO%d INB=GPIO%d PWM=GPIO%d at %d/1024 counts for 5 s\n",
           argv[1], en, ina, inb, pwm, counts);
    digitalWrite(en, HIGH); digitalWrite(ina, HIGH); digitalWrite(inb, LOW);
    pwmWrite(pwm, counts);
    sleep(5);
    all_off();
    printf("off\n");
    return 0;
}
