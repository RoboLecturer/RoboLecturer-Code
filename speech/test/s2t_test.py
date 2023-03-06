from speech.pkg.speech2text import *

filename = "6241-61943-0027.flac"

# List available mics
print(sr.Microphone.list_microphone_names())

# Test with audio file
def s2tAudio(filename):
    with sr.AudioFile(filename) as source:
        # listen for the data (load audio to memory)
        audio_data = r.record(source)
        # recognize (convert from speech to text)
        text = Sphinx(audio_data)
        text = GSR(audio_data)
    return text

# print(s2tAudio)
print(runS2T)