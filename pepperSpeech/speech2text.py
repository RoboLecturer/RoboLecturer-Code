#pip install SpeechRecognition pydub

import speech_recognition as sr
filename = "6241-61943-0027.flac"

r = sr.Recognizer()

# open the file
with sr.AudioFile(filename) as source:
    # listen for the data (load audio to memory)
    audio_data = r.record(source)
    # recognize (convert from speech to text)
    text = r.recognize_google(audio_data)
    print(text)
    
#################
#offline version:
    
# from pocketsphinx import AudioFile
# audio = AudioFile("/content/6241-61943-0027.flac",lm=False, keyphrase='forward', kws_threshold=1e-20)

# for phrase in audio: 
#   print(phrase) 