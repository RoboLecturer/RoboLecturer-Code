# pip install TTS
# brew install espeak
from TTS.api import TTS

OUTPUT_PATH = "output.wav"
INPUT_PATH = "text.txt"

with open(INPUT_PATH, 'r') as file:
    txt = file.read().rstrip()

# List available 🐸TTS models and choose the first one
# model_name = TTS.list_models()[0]

def runT2S(txt):
    # Init TTS with the target model name
    # print(TTS.list_models())
    tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=True, gpu=False)
    # Text to speech with a numpy output
    # wav = tts.tts(txt, speaker=tts.speakers[0], language=tts.languages[0])
    # Run TTS
    tts.tts_to_file(text=txt, file_path=OUTPUT_PATH)

    return OUTPUT_PATH

#runT2S(txt)