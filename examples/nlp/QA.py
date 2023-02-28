# Script for question processing and answer generation 

#!/usr/bin/env python
#using Hugging Face pipeline module
from transformers import pipeline

#create a default pretrained model for QA
#now the default one is DistilBERT-base pretrained on the SQuAD dataset
#to use our own model and tokenizer, just pass them as parameters to the pipeline
question_answering = pipeline("question-answering")

#we need a context, which contains the answer of a question
#context = """
#Machine learning (ML) is the study of computer algorithms that improve automatically through experience. It is seen as a part of artificial intelligence. Machine learning algorithms build a model based on sample data, known as "training data", in order to make predictions or decisions without being explicitly programmed to do so. Machine learning algorithms are used in a wide variety of applications, such as email filtering and computer vision, where it is difficult or unfeasible to develop conventional algorithms to perform the needed tasks.
#"""
#question = "What are machine learning models based on?"

context = """Apple is a fruit."""
question = "Hey Pepper. What is apple?"

#model inference
result = question_answering(question=question, context=context)

#answer
print("Answer:", result['answer'])

#confidence score
print("Score:", result['score'])

#by defining context as the content of lecture slides and
#question as the text converted from the speech of a student, now we can have a very simple QA system