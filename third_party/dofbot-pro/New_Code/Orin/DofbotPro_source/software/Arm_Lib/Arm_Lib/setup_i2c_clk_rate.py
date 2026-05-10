import subprocess
import time


# subprocess.run("echo 100000 > /sys/class/i2c-adapter/i2c-7/bus_clk_rate", shell=True)

time.sleep(3)

try:
    file_path = '/sys/class/i2c-adapter/i2c-7/bus_clk_rate'
    result = subprocess.run(['cat', file_path], stdout=subprocess.PIPE, text=True)
    output = str(result.stdout)
    print("read i2c-7 bus clk rate:", output)
    if int(output) != 100000:
        subprocess.run("echo 100000 > /sys/class/i2c-adapter/i2c-7/bus_clk_rate", shell=True)
        print("set i2c-7 bus clk rate: 100000")
except:
    print("set i2c-7 bus clk rate error")
