import time

LED = "/sys/class/leds/uvled/"

def main():
    with open(LED + "trigger", "w") as f:
        f.write("none\n")
        f.flush()
        
    with open(LED + "brightness", "w") as f:
        while True:
            f.write("1\n")
            f.flush()
            time.sleep(1)
            f.write("0\n")
            f.flush()
            time.sleep(1)
            
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        with open(LED + "brightness", "w") as f:
            f.write("0\n")
            f.flush()
        with open(LED + "trigger", "w") as f:
            f.write("heartbeat\n")
            f.flush()