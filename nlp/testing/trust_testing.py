# imput 55 classic engingeering interview questions and evaluate how chat models answer the question
from chat import chat_model, manage
from time import time, sleep
from tqdm import tqdm
from collections import deque

class FixedLengthQueue:
    def __init__(self, max_len):
        self.queue = deque(maxlen=max_len)

    def append(self, item):
        self.queue.append(item)

    def __str__(self):
        return str(list(self.queue))

# define the conversation 
conversation = FixedLengthQueue(25)
Context = "I am in an interview for a job in machine learning. My goal is to fully express my understanding of computer science and the world of machine learning in a concise and likeable manner."
conversation.append({'role': 'system', 'content': Context})

# define the questions
questions = [
    "Define deep learning. How is it different from other machine learning algorithms?",
    "Is model accuracy or model performance more important to you?",
    "Explain how you would ensure you don’t overfit a model.",
    "What is a hash table?",
    "Explain what you believe is our business’s most valuable data.",
    "Could you name a few machine learning papers you’ve read recently or explain how you follow the latest developments in machine learning?",
    "Explain how you would reproduce AlphaGo’s approach at Go that beat Lee Sedol.",
    "Do you think quantum computing will affect machine learning? How?",
    "What research experience do you have in machine learning?",
    "What data types does JSON support? ",
    "List some differences between an array and a linked list.",
    "Explain how you would assess a logistic regression model.",
    "When should you use classification over regression?",
    "How would you prune a decision tree?",
    "What’s your favorite algorithm? Give me a simple explanation.",
    "Explain the difference between supervised and unsupervised machine learning.",
    "What is a Fourier transform?",
    "When assessing if a machine learning model is effective, what evaluation approaches would you take?",
    "Write down the pseudo-code for a parallel implementation of your choice of algorithm.",
    "Is it possible to cut two strings, A and B, that are the same length at a common point so that the first section of A and the second section of B create a palindrome?",
    "Explain how you would implement a recommendation system for our company’s customers",
    "Where would you typically source datasets from?",
    "Have you trained models for fun? What hardware or graphics processing units did you use?",
    "How would you approach the ‘Netflix Prize’ competition?",
    "Explain how primary and foreign keys are linked in SQL.",
    "Have you used Spark or other big data tools? ",
    "When do you think ensemble techniques might be practical?",
    "Explain the difference between a discriminative and a generative model.",
    "How is L1 regularization different from L2?",
    "What is precision? What is recall?",
    "Explain the trade-off between variance and bias.",
    "What are some of your favorite APIs to explore?",
    "Explain how XML compares to CSVs in terms of size.",
    "If you were given an imbalanced dataset, how would you handle it?",
    "What do you think about the GPT-3 model?",
    "What are your thoughts on how Google is training data for self-driving cars?",
    "How would you build a data pipeline?",
    "List some data visualization libraries you’ve used. What data visualization tools do you think are the best?",
    "What would you do if you discovered missing or corrupted data in a dataset?",
    "Define the F1 score. How would you use it?",
    "Explain the difference between Type I and Type II errors.",
    "How does a ROC curve work?",
    "Explain how your machine learning skills will help our company generate profits.",
    "Give me examples of your favorite machine learning models.",
    "What are your thoughts on our data process?",
    "In machine learning, what are the three model-building stages?",
    "Explain the differences between machine and deep learning.",
    "List some supervised machine learning applications used in modern businesses.",
    "Explain the differences between inductive and deductive machine learning.",
    "How would you choose which algorithm you will use for a classification problem?",
    "What are your thoughts on Amazon’s recommendation engine? How does it work?",
    "Define Kernel SVM.",
    "Explain how you would build an email spam filter.",
    "Explain what a recommendation system is.",
    "Considering that many machine learning algorithms exist, how would you choose an algorithm for a particular dataset?"
]

for question in tqdm(questions):

    flat = manage.flatten_convo(conversation)
    # get the anticipation
    query_anticipate = f"Given the following chat log, inger the user's actual information needs. Attempt to anticipate what the user truly needs even if the user doesn not fully understand it yet themselves, or is asking the wrong questions.\n\n {flat}"
    anticipation = chat_model.getResponse(query_anticipate)
    
    # get the salience 
    query_salience = f"Given the following chat log, write a brief summary of only the most salient points of the conversation:\n\n {flat}"
    salience = chat_model.getResponse(query_salience)
    
    # update the conversation context with the salience and anticipation for the current point in the conversation
    conversation[0]['content'] = Context + "I am in the middle of a conversation: %s. I anticipate the user needs: %s. I will do my best to fulfill my objectives." % (salience, anticipation)

    # combine all elements in the list conversationHistory
    convHistory = manage.flatten_convo(conversation)
    query = f"give a short answer to the following queston: {question} when the context: {convHistory}"
    response = chat_model.getResponse(query)

    conversation.append({'role': 'user', 'content': question})
    conversation.append({'role': 'assistant', 'content': response})
    
    sleep(2)

filepath = "logs/trust_%s.txt" % round(time(),4)
output = manage.flatten_image(conversation)
with open(filepath, 'w', encoding='utf-8') as outfile:
    outfile.write(output)




