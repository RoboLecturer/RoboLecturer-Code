# This script is where the calls to the openai models are

import openai

def getModel():
    """get the current model under use
    @returns: model [string]
    """
    # model = "chatgpt"
    model = "davinci"
    return model

def chatGPT(query):
    """call the openai chatGPT api
    @params: query [string]
    @reutrns: response [string]
    """
    openai.api_key = "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"
    # create completion
    completions = openai.ChatCompletion.create(
		    model="gpt-3.5-turbo",
		    messages=[
				    {"role": "system", "content": f"{query}"},
			    ]
	    )
    response = completions['choices'][0]['message']['content']
    return response

def daVinci(query):
    """call the openai daVinci api
    @params: query [string]
    @returns: response [string]
    """
    openai.api_key = "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"
    # create completion
    completions = openai.Completion.create(
		engine = "text-davinci-003",
		prompt = f"{query}",
		max_tokens = 1024,	
		n = 1,
		temperature = 0.2,
	)
    response = completions.choices[0]["text"]
    return response

def getResponse(query):
    """get a response to the query using the selected chat model
    @params: query [string]
    @returns: response [string]
    """
    model = getModel()
    if model == "chatgpt":
        response = chatGPT(query)
    elif model == "davinci":
        response = daVinci(query)
    return response