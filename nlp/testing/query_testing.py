# This script will allow you set a number of querys and then test each one for relatabilty

# import the packges
import time 
from chat import chat_model

#define output class
class QA:
    query = ""
    response = ""
    time = ""
    def __init__(self, query):
        self.query = query

# set the list of queries 
query_instances = []

# define the topics
#topic = "Arsenal Football Club"
topic = "Astronomy"

# define the context of the queries
context_query = f"Write a lecture script of 4 paragraphs, covering a small range of points on this topic: {topic}"
context = chat_model.getResponse(context_query)

querys = []

#define some questions
prompt = f"give me 5 questions (ending with a ? and seperated by a new line) based on the topic: {topic}"
query = chat_model.getResponse(prompt)
querys = [x for x in query.split("\n") if x != '']
print("====================================================\n")
print(querys)
print("====================================================\n")

for query in querys:
    query_instances.append(QA(query))

# create completions for each of the queries
for instance in query_instances:
    start_time = time.time()
    query = f"give a breif by complete answer to this question - {instance.query} - that relates to this context: {context}"
    # query = f"answer this question: {instance.query} - in this context of this text: {context}, if the answer is not in the context of the text, then provide an answer that relates to the topic contained within the text" 
    instance.response = chat_model.getResponse(query)
    end_time = time.time()
    instance.time = end_time - start_time
    # implement time delays so that we don't ciolate the requests/min
    time.sleep(5)


# print the results along with the question for judging
print(f"Query Context:\n{context}")
print("=========================================================\n")
for instance in query_instances:
    print("====================================================\n")
    print(f"Query: {instance.query}\nAnswer: {instance.response}\nExecution Time: {instance.time}\n")
    print("====================================================\n")





# how could we validate whether it is a good answer?
