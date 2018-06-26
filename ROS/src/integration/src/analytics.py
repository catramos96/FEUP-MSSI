import time
import json

'''
Class to start recording times
'''


class Timer:

    start_time = 0
    time_list = []


def start():
    Timer.start_time = time.time()


def end():
    end = time.time()
    diff = end - Timer.start_time
    Timer.time_list.append(diff)
    save()


def save():
    f = open("responseTimes.txt", "w")
    json.dump(Timer.time_list, f)
    f.close()
