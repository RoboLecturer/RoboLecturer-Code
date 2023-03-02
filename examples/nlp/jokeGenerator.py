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
    completions = openai.Completion.create(
        engine="text-davinci-003",
        prompt=query,
        max_tokens=1024,
        n=1, # generate a single completion
        temperature=0.2, # keeps responses narrow
    )
    # retrive response
    response = completions.choices[0]["text"]
    # return response
    return response