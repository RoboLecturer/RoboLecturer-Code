import openai
import time

openai.api_key = "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"
start_time = time.time()
response = openai.ChatCompletion.create(
  model="gpt-3.5-turbo",
  messages=[
        {"role": "system", "content": "What is Arsenal Football club?"},
    ]
)
end_time = time.time()

print(response['choices'][0]['message']['content'])
print(f"Execution Time: {end_time - start_time}")

