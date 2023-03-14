# This script will allow you manually ask questions to the AI lecturer and validate the answer of these queries. 

# import the packges
from chat import chat_model, manage
from time import time

#define output class
class QA:
    query = ""
    response = ""
    time = ""
    def __init__(self, query):
        self.query = query

conversationHistory = list()
pepperContext = "I am an AI Teacher and lecturer. I have 5 goals: teach my students the lesson plan I am given, answer their questions to clear up areas of ambiguity, ask them questions to gauge understanding  and quiz them, maintain order in the classroom, and be ultimately helpful."
conversationHistory.append({'role': 'system', 'content': pepperContext})

# set the list of queries 
query_instances = []

# define the topics
#topic = "Arsenal Football Club"
topic = "Picasso"

# define the context of the queries
context_query = f"Write a lecture script of 3 paragraphs, covering a small range of points on this topic: {topic}"
context = chat_model.getResponse(context_query)
# add the lecture script to the conversation history
conversationHistory.append({'role': 'assistant', 'content': context})



print("====================================================\n")
print(topic)
print("====================\n")
print(context)
print("====================================================\n")


while True:
    a = input('\n\nUSER: ')
    if a == "bye":
        break
    conversationHistory.append({'role': 'user', 'content': a})

    flat = manage.flatten_convo(conversationHistory)
    query_anticipate = f"Given the following chat log, inger the user's actual information needs. Attempt to anticipate what the user truly needs even if the user doesn not fully understand it yet themselves, or is asking the wrong questions.\n\n {flat}"
    anticipation = chat_model.getResponse(query_anticipate)
    query_salience = f"Given the following chat log, write a brief summary of only the most salient points of the conversation:\n\n {flat}"
    salience = chat_model.getResponse(query_salience)

    conversationHistory[0]['content'] = pepperContext + "I am in the middle of a conversationL %s. I anticipate the user needs: %s. I will do my best to fulfill my objectives." % (salience, anticipation)

    convHistory = manage.flatten_convo(conversationHistory)
    query = f"give a breif but complete answer to this question - {a} - that relates to this context: {convHistory}"

    response = chat_model.getResponse(query)
    print(response)
    print("==============\n")

    conversationHistory.append({'role': 'user', 'content': a})
    conversationHistory.append({'role': 'assistant', 'content': response})


filepath = "logs/conversation_%s.txt" % round(time(),4)

output = manage.flatten_convo(conversationHistory)
with open(filepath, 'w', encoding='utf-8') as outfile:
    outfile.write(output)

