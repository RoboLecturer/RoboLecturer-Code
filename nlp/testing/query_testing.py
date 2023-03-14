# This script will allow you set a number of querys and then test each one for relatabilty

# import the packges
import time 
from chat import chat_model, manage
from tqdm import tqdm 

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
context_query = f"Write a lecture script of 5 paragraphs, covering a small range of points on this topic: {topic}"
context = chat_model.getResponse(context_query)
# add the lecture script to the conversation history
conversationHistory.append({'role': 'assistant', 'content': context})

#define some questions
prompt = f"give me 5 questions (ending with a ? and seperated by a new line) based on the topic: {topic}"
query = chat_model.getResponse(prompt)
querys = [x for x in query.split("\n") if x != '']
prompt = f"give me 2 questions that require a subjective answer (ending with a ? and seperated by a new line) based on the topic: {topic}"
subjective_query = chat_model.getResponse(prompt)
subjective_querys = [x for x in subjective_query.split("\n") if x != '']

print("====================================================\n")
print(topic)
print("====================\n")
print(querys)
print("====================\n")
print(subjective_querys)
print("====================================================\n")

querys.extend(subjective_querys)

for query in querys:
    query_instances.append(QA(query))

# create completions for each of the queries
for instance in tqdm(query_instances):

    ant_sal_history = manage.flatten_convo(conversationHistory)
    # get the anticipation
    query_anticipate = f"Given the following chat log, inger the user's actual information needs. Attempt to anticipate what the user truly needs even if the user doesn not fully understand it yet themselves, or is asking the wrong questions.\n\n {ant_sal_history}"
    anticipation = chat_model.getResponse(query_anticipate)
    
    # get the salience 
    query_salience = f"Given the following chat log, write a brief summary of only the most salient points of the conversation:\n\n {ant_sal_history}"
    salience = chat_model.getResponse(query_salience)
    
    # update the conversation context with the salience and anticipation for the current point in the conversation
    conversationHistory[0]['content'] = pepperContext + "I am in the middle of a conversation: %s. I anticipate the user needs: %s. I will do my best to fulfill my objectives." % (salience, anticipation)

    # combine all elements in the list conversationHistory
    convHistory = manage.flatten_convo(conversationHistory)
    query = f"give a breif but complete answer to this question - {instance.query} - that relates to this context: {convHistory}"

    start_time = time.time()
    instance.response = chat_model.getResponse(query)
    instance.time = time.time() - start_time

    conversationHistory.append({'role': 'user', 'content': instance.query})
    conversationHistory.append({'role': 'assistant', 'content': instance.response})
    # implement time delays so that we don't ciolate the requests/min
    time.sleep(5)


# print the results along with the question for judging
print(f"Query Context:\n{context}")
print("=========================================================\n")
count = 1
for instance in query_instances:
    if count < 6:
        print("====================================================\n")
        print(f"Query: {instance.query}\nAnswer: {instance.response}\nExecution Time: {instance.time}\n")
        print("====================================================\n")
    elif count > 5:
        print("====================================================\n")
        print(f"SUBJECTIVE Query: {instance.query}\nAnswer: {instance.response}\nExectution Time: {instance.time}\n")
    count=+1

filepath = "logs/QandA_%s.txt" % round(time.time(),4)
output = manage.flatten_convo(conversationHistory)
with open(filepath, 'w', encoding='utf-8') as outfile:
    outfile.write(output)

# how could we validate whether it is a good answer?
