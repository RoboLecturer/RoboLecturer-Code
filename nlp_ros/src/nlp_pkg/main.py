#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info
import QA

def main():

	question = Info.Request("Question")

	answer = QA.question_answering(question)

	Info.Send("Answer", {"text": answer})