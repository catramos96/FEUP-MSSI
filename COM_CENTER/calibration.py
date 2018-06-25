import datetime
import messages
import time

def calibration(controllers, time_period_seconds):
    last_checked_time = datetime.datetime.now()

    while(1):
        elapsed = (datetime.datetime.now() - last_checked_time).total_seconds()
        if(elapsed >= time_period_seconds):
            
            for i in range(0,len(controllers)):
                controllers[i].send_message(messages.getCalibrationMessage(controllers[i].car_id,datetime.datetime.now()))

            last_checked_time = datetime.datetime.now()
            
        else:
            time.sleep(time_period_seconds - elapsed)