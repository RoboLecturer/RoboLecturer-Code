# pip install gTTS
# pip install TTS
# brew install espeak
from TTS.api import TTS
from gtts import gTTS

def DNNTTS(txt, OUTPUT_PATH):
    # Init TTS with the target model name
    # print(TTS.list_models())
    # Curr: Tacotron2-DDC + HiFiGAN
    tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=True, gpu=False)
    # Run TTS
    tts.tts_to_file(text=txt, file_path=OUTPUT_PATH)

def googleTTS(txt, OUTPUT_PATH):
    tts = gTTS(txt)
    tts.save(OUTPUT_PATH)

def runT2S(txt, online=False, OUTPUT_PATH="output"):
    if online:
        OUTPUT_PATH += '.mp3'
        googleTTS(txt, OUTPUT_PATH)
    else:
        OUTPUT_PATH += '.wav'
        DNNTTS(txt, OUTPUT_PATH)

    return OUTPUT_PATH