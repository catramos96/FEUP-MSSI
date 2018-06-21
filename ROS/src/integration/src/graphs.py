import matplotlib.pyplot as plt
import json

#try:
f = open("responseTimes.txt", "r")
times_list = json.load(f)
f.close()
x_list = range(1, len(times_list)+1)
    
plt.plot(x_list, times_list, "ro")
plt.show()
#except:
 #   print("no file")

