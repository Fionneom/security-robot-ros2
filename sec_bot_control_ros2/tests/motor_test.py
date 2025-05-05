import lgpio
import time

PWMA_PIN = 18
AIN2_PIN = 23
AIN1_PIN = 24

h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, AIN1_PIN)
lgpio.gpio_claim_output(h, AIN2_PIN)

lgpio.gpio_write(h, AIN1_PIN, 0)
lgpio.gpio_write(h, AIN2_PIN, 1)

lgpio.gpio_claim_output(h, PWMA_PIN)

speed = 0
direction = 1

lgpio.tx_pwm(h, PWMA_PIN, 8000, 50)

time.sleep(5)

lgpio.tx_pwm(h, PWMA_PIN, 8000, 0)

