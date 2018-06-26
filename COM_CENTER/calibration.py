import datetime
import messages
import time

'''
Thread: calibrates the delay between the two environments after a time_period
'''
def calibration(controllers, time_period_seconds):

    while(1):

        for i in range(0,len(controllers)):
            controllers[i].send_message(messages.getCalibrationMessage(controllers[i].car_id,datetime.datetime.now()))

        time.sleep(time_period_seconds)