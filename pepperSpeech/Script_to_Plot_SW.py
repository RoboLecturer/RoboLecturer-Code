import matplotlib.pyplot as plt
import numpy as np
import wave
import sys
from math import log10

# m4a_file = r"/Users/hazemmassoud/Desktop/SPEAKERS/rode 2.mp4"
# wav_filename = r"/Users/hazemmassoud/Desktop/SPEAKERS/rode 2.wav"
# from pydub import AudioSegment
# track = AudioSegment.from_file(m4a_file,  format= 'm4a')
# file_handle = track.export(wav_filename, format='wav')

# def db(i):
#     if i > 0 :
#         return 20*log10(i)
#     else:
#         return 0

spf = wave.open("/Users/hazemmassoud/Desktop/SPEAKERS/ReSpeaker 2.wav", "r")

# Extract Raw Audio from Wav File
signal = spf.readframes(-1)
signal = np.frombuffer(signal, dtype=np.int16)
# apply=np.vectorize(db)
# final = apply(signal)
fs = spf.getframerate()

# If Stereo
# if spf.getnchannels() == 2:
#     print("Just mono files")
#     sys.exit(0)


Time = np.linspace(0, 3, num=len(signal))

plt.figure(1)
plt.title("Signal Wave")
plt.plot(Time, signal)
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.show()

