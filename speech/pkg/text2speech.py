# pip install gTTS
# pip install TTS
# pip install pydub
# brew install espeak, ffmpeg
from TTS.api import TTS
from gtts import gTTS

from pydub import AudioSegment

def DNNTTS(txt, OUTPUT_PATH, GPU=False):
    # Init TTS with the target model name
    # print(TTS.list_models())
    # Curr: Tacotron2-DDC + HiFiGAN
    tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=True, gpu=GPU)
    # Run TTS
    tts.tts_to_file(text=txt, file_path=OUTPUT_PATH)

def googleTTS(txt, OUTPUT_PATH):
    tts = gTTS(txt)
    tts.save(OUTPUT_PATH)

def runT2S(txt, online=False, speedup=True, OUTPUT_PATH="output"):
    if online:
        filetype = 'mp3'
        OUTPUT_PATH += '.'+filetype
        googleTTS(txt, OUTPUT_PATH)
    else:
        filetype = 'wav'
        OUTPUT_PATH += '.'+filetype
        DNNTTS(txt, OUTPUT_PATH)
    
    if speedup:
        slow_mp3 = AudioSegment.from_file(OUTPUT_PATH, format=filetype)
        speed_update = slow_mp3.speedup(1.2)
        speed_update.export(OUTPUT_PATH, format=filetype)

    return OUTPUT_PATH