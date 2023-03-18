# script to test the ada call for embeddings

import openai

openai.api_key = "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"

content = "hello world"
print(f"String for embedding: {content}")

content = content.encode(encoding='ASCII',errors='ignore').decode()
response = openai.Embedding.create(input=content, engine='text-embedding-ada-002')
embedding = response['data'][0]['embedding']

print(f"Embedded string from ada: {embedding[1]} - which is of type {type(embedding)}")


