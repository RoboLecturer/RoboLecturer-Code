#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info
import QA
import chatgpt
import davinci

PepperAPI.init("nlp")
question = Info.Request("Question")

#answer = QA.question_answering(question)
#answer = chatgpt.asking_chatgpt(question)
#answer = davinci.asking_davinci(question)
answer = chatgpt.asking_chatgpt("What is apple?")

Info.Send("Answer", {"text": answer})
