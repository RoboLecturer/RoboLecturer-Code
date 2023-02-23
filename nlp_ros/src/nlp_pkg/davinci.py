import os
import openai
from datetime import datetime

openai.api_key = os.getenv("OPENAI_API_KEY")

def asking_davinci(question):
    
    start_time = datetime.now()   

    response = openai.Completion.create(
      model="text-davinci-003",
      prompt=question,
      temperature=0,
      max_tokens=100,
      top_p=1,
      frequency_penalty=0.0,
      presence_penalty=0.0,
    )
    answer = response["choices"][0]["text"].strip(" \n")

    end_time = datetime.now()

    print("Response: ", response)
    print("Answer: ", answer)
    print('Time required: ', end_time - start_time)

#asking_davinci("What is apple?")
