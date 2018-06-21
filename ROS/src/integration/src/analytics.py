import time
import json

class Timer:

    time_list = []

    def start(self):
        self.start_time = time.time()


    def end(self):
        end = time.time()
        diff = end - self.start_time
        self.time_list.append(diff)
        self.save()

    def save(self):
        f = open("responseTimes.txt", "w")
        json.dump(self.time_list, f)
        f.close()
