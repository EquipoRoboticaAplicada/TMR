from util import ImgProcessor, Sender

PI_IP = "172.32.145.45"

if __name__ == "__main__":
    sender  = Sender(PI_IP).start()
    grabber = ImgProcessor(PI_IP).start()

    try:
        grabber.run_tracker(sender)    
    finally:
        grabber.stop()
        sender.stop()