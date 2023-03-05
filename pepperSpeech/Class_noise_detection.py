import pyaudio
import time
from math import log10
import audioop  

#initiate object
p = pyaudio.PyAudio()

#set paramaters needed. 
WIDTH = 2                                                           #Bit_Size of audio (2 bytes- 16 bits)
RATE = int(p.get_default_input_device_info()['defaultSampleRate']) #getting the default sample rate of the mic being used
MIC = p.get_default_input_device_info()['index']                    #setting the mic being used to a variable Mic
rms = 1                                                             #Initiating RMS
db=0                                            
print(p.get_default_input_device_info())

#Define a callback function 

def callback(in_data, frame_count, time_info, status):
   global rms
   rms = audioop.rms(in_data, WIDTH) 
   return in_data, pyaudio.paContinue

#Open A stream of PyAudio to start listening

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

while stream.is_active(): 
    if rms != 0:
        db = 20 * log10(rms)
    print(f"RMS: {rms} DB: {db}") 
    # refresh every 0.3 seconds 
    time.sleep(0.3)
    

# stream.stop_stream()
# stream.close()

# p.terminate()