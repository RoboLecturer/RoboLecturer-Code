#!/usr/bin/env python3

from PepperAPI import Info
import QA
import chatgpt

question = Info.Request("Question")

#answer = QA.question_answering(question)
#answer = chatgpt.asking_chatgpt(question)
answer = chatgpt.asking_chatgpt("What is apple?")

Info.Send("Answer", {"text": answer})
