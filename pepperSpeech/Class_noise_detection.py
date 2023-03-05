import pyaudio
import time
from math import log10
import audioop  
import time
import matplotlib as plt
#initiate object
p = pyaudio.PyAudio()

#set paramaters needed. 
MIC = 2
WIDTH = 2                                                           #Bit_Size of audio (2 bytes- 16 bits)
RATE = int(p.get_device_info_by_index(MIC)['defaultSampleRate']) #getting the default sample rate of the mic being used]                  #setting the mic being used to a variable Mic
rms = 1                                                            #Initiating RMS
db=0                                            
#print(p.get_default_input_device_info())

#Define a callback function 

def callback(in_data, frame_count, time_info, status):
   global rms
   rms = audioop.rms(in_data, WIDTH) 
   return in_data, pyaudio.paContinue

# #Open A stream of PyAudio to start listening

stream = p.open(format=p.get_format_from_width(WIDTH),
                input_device_index=MIC,
                channels=1,
                rate=RATE,
                input=True,
                output=False,
                stream_callback=callback)

   
stream.start_stream()

#While loop to calculate db of noise, print it
#and refresh every 0.3 seconds



def Shush(limit,timel):
    start_time = time.time()
    elapsed_time = 0
    while elapsed_time < timel: 
        if rms != 0:
            db = 20 * log10(rms)
        print("db is {}".format(db))
        if db >= limit:
            return True
        
        elapsed_time = time.time() - start_time
        #print(elapsed_time)
        # refresh every 0.3 seconds 
        time.sleep(0.3)
    return False
        
#print(Shush(70, 10))
# stream.stop_stream()
# stream.close()

# p.terminate()