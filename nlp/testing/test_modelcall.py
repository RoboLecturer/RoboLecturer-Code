# test the call back 

from chat import chat_model

model = chat_model.getModel()
print(model)

query = f"In two sentences, describe arsenal football club"

response = chat_model.getResponse(query)

print(response)