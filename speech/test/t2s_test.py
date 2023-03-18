import sys
sys.path.append("..") 
from pkg.text2speech import *

# INPUT_PATH = "text.txt"
# with open(INPUT_PATH, 'r') as file:
#     txt = file.read().rstrip()

txt = "This is a test sentence to split. Here's another one for you"
sentences = [x for x in txt.split('.') if x]

# List available 🐸TTS models and choose the first one
# model_name = TTS.list_models()[0]

for i in range(len(sentences)):
    runT2S(sentences[i], online=True, OUTPUT_PATH=f"sentences/output_test_{i}")