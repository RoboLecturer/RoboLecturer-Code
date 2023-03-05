#pip install SpeechRecognition pyaudio

import speech_recognition as sr

filename = "6241-61943-0027.flac"

def Sphinx(audio):
    # recognize speech using Sphinx
    try:
        return "Sphinx thinks you said " + r.recognize_sphinx(audio)
    except sr.UnknownValueError:
        print("Sphinx could not understand audio")
    except sr.RequestError as e:
        print("Sphinx error; {0}".format(e))

def GSR(audio): 
    # recognize speech using Google Speech Recognition
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        print("Google Speech Recognition thinks you said " + r.recognize_google(audio))
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def whisper(audio): 
# recognize speech using whisper
    try:
        print("Whisper thinks you said " + r.recognize_whisper(audio, language="english"))
    except sr.UnknownValueError:
        print("Whisper could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Whisper")


r = sr.Recognizer()

# List available mics
print(sr.Microphone.list_microphone_names())

# Test with mic
def runSTT(index):
    with sr.Microphone(device_index=index) as source:
        #print("Say something!")
        mic_input = r.listen(source)
        text = Sphinx(mic_input)
        # GSR(mic_input)
    return text

# Test with audio file
#with sr.AudioFile(filename) as source:
    # listen for the data (load audio to memory)
 #   audio_data = r.record(source)
    # recognize (convert from speech to text)
  #  Sphinx(audio_data)
   # GSR(audio_data)
    # text = r.recognize_google(audio_data)
    # print(text)