import requests
import json
import time

def get_chat_completion():
	url = "https://api.openai.com/v1/chat/completions"
	headers = {
		"Authorization": "Bearer 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",
		"Content-Type": "application/json"
	}
	data = {
		"model": "gpt-3.5-turbo",
		"messages": [{"role": "user", "content": "What is Arsenal football club?"}]
	}
	response = requests.post(url, headers=headers, data=json.dumps(data))
	return response.json()

num_iters = 20
sum = 0
for i in range(num_iters):
	start_time = time.time()
	response = get_chat_completion()
	end_time = time.time()
	sum += end_time - start_time

av_time = sum/num_iters 

print(response["choices"][0]["message"]["content"])
print(f"Average execution time: {av_time}")
