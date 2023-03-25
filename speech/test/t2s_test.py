import sys
sys.path.append("..") 
from pkg.text2speech import *

# INPUT_PATH = "text.txt"
# with open(INPUT_PATH, 'r') as file:
#     txt = file.read().rstrip()

txt = "Good morning everybody. It’s nice to see such a big audience. When I was asked to give a topic to give a talk, on a topic, that was of interest to both scientists and non scientists, and I thought for a few minutes and then I thought one subject that is dear to many people’s hearts is chocolate. Is there anybody here who doesn’t like chocolate? Put your hand up if you don’t. Oh yeah, the odd one, but not many, and indeed there have been surveys that the population about how much they like chocolate, this is a recent survey from women in the United States, and that shows that the average consumption of chocolate is more than five kilos per person per year. More than 93 percent of the population eat chocolate. Three percent feel guilty about eating chocolate. So chocolate - Some people feel they shouldn’t enjoy it but they do. 67 percent prefer milk chocolate to dark chocolate. And 52 percent feel that eating chocolate makes them feel happy. So some people turn to chocolate when they are feeling a bit down and they eat a bar of chocolate and it makes them feel happy. So what I want to do today is to talk to you a bit about where chocolate comes from, how it’s made, a bit about the history of chocolate making, and a bit about the quality attributes, what manufactures think consumers require when they buy a bar of chocolate."
sentences = [x for x in txt.split('.') if x]

# List available 🐸TTS models and choose the first one
# model_name = TTS.list_models()[0]

# for i in range(len(sentences)):
#     runT2S(sentences[i], online=True, speedup=True, OUTPUT_PATH=f"sentences/output_test_fast_{i}")

runT2S(txt, online=True, speedup=True, OUTPUT_PATH=f"output_test_fast")