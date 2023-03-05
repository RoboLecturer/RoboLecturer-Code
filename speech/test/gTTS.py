# pip install gTTS
from gtts import gTTS

OUTPUT_PATH = "output.mp3"
INPUT_PATH = "text.txt"

with open(INPUT_PATH, 'r') as file:
    txt = file.read().rstrip()

def runT2S(txt):
    tts = gTTS(txt)
    tts.save(OUTPUT_PATH)

    return OUTPUT_PATH

runT2S(txt)