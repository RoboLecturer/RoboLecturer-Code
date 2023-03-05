#pip install SpeechRecognition pyaudio

import speech_recognition as sr
import PepperAPI
from PepperAPI import info
from speech2text import *


filename = "6241-61943-0027.flac"

if __name__ == "__main__":
    #initialize API
    PepperAPI.init("speech_module")
    script = Info.Request("LectureScript")
    #run t2s
    
    # List available mics
    #print(sr.Microphone.list_microphone_names())

    # Test with mic
    r = sr.Recognizer()
    with sr.Microphone(device_index=0) as source:
        print("Say something!")
        mic_input = r.listen(source)
        question = Sphinx(mic_input)
        # GSR(mic_input)
        #print(text)
    Info.send("Question", {"text": question})
    answer = Info.Request("Answer")
    #run t2s with answer


# Test with audio file
#with sr.AudioFile(filename) as source:
    # listen for the data (load audio to memory)
 #   audio_data = r.record(source)
    # recognize (convert from speech to text)
  #  Sphinx(audio_data)
   # GSR(audio_data)
    # text = r.recognize_google(audio_data)
    # print(text)