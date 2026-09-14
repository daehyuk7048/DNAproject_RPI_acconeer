python - <<'PY'
import rgpio, time
PI_IP = "172.29.15.227"
PORT  = 8889
s = rgpio.sbc(PI_IP, PORT)
h = s.gpiochip_open(0)
GPIO = 14
s.gpio_claim_output(h, GPIO)
s.gpio_write(h, GPIO, 1); time.sleep(1.0)
s.gpio_write(h, GPIO, 0)
s.gpiochip_close(h); s.stop()
print("OK")
PY