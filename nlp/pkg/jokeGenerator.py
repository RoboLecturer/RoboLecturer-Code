# This scirpt generates a joke based on different requirements #

#  import packages
import openai

# function the generate a joke
def genJoke(jokeType):
    """This function generate a joke depending on the type of joke required
    Args: jokeType - [string] The type of joke is dictated by the reason for the joke
            "noiseHigh"
            "attentionLow"
    Returns: response - [string] Return the joke
    """

    # define the API key
    openai.api_key = "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"

    # define the query depending on the type
    if jokeType == "noiseHigh":
        query = f"come up with a short, funny joke to retain the concentration of a noisy classroom"
    elif jokeType == "attentionLow":
        query = f"come up with a short, funny joke to entertain a classrom of students"
    else:
        query = f"come up with a short, funny joke for me to tell my students"

    #  create completion
    completions = openai.ChatCompletion.create(
		    model="gpt-3.5-turbo",
		    messages=[
				    {"role": "system", "content": f"{query}"},
			    ]
	    )
    response = completions['choices'][0]['message']['content']
    # return response
    return response