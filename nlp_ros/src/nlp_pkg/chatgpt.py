from chatgpt_wrapper import ChatGPT
from datetime import datetime

def asking_chatgpt(question):
    bot = ChatGPT()
    start_time = datetime.now()
    response = bot.ask(question)
    print(response)  # prints the response from chatGPT
    end_time = datetime.now()
    print('time required: ', end_time - start_time)

    return response

#asking_chatgpt("What is apple?")