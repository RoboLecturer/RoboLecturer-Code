# pip install TTS
# brew install espeak
from TTS.api import TTS

def runT2S(txt, OUTPUT_PATH="output.wav"):
    # Init TTS with the target model name
    # print(TTS.list_models())
    # Curr: Tacotron2-DDC + HiFiGAN
    tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=True, gpu=False)
    # Run TTS
    tts.tts_to_file(text=txt, file_path=OUTPUT_PATH)

    return OUTPUT_PATH