import openai

openai.api_key = "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"

response = openai.ChatCompletion.create(
  model="gpt-3.5-turbo",
  messages=[
        {"role": "system", "content": "How are you doing?"},
    ]
)
print(response['choices'][0]['message']['content'])
