from speech.pkg.text2speech import *

INPUT_PATH = "text.txt"

with open(INPUT_PATH, 'r') as file:
    txt = file.read().rstrip()

# List available 🐸TTS models and choose the first one
# model_name = TTS.list_models()[0]

runT2S(txt, OUTPUT_PATH="output_test")