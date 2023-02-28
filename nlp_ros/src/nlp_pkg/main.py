#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info
import QA
import chatgpt
import davinci
import ScriptGenerator

PepperAPI.init("nlp")
question = Info.Request("Question")

#answer = QA.question_answering(question)
#answer = chatgpt.asking_chatgpt(question)
#answer = davinci.asking_davinci(question)
answer = chatgpt.asking_chatgpt("What is apple?")

Info.Send("Answer", {"text": answer})

# Generate script
slides = Info.Request("Slides")
script = ScriptGenerator.createScript(slides)
# return script
Info.Send("LectureScript", {"text: script"})
