# This scirpt generates a joke based on different requirements #

#  import packages
from pkg.chat import chat_model

# function the generate a joke
def genJoke(jokeType):
    """This function generate a joke depending on the type of joke required
    Args: jokeType - [string] The type of joke is dictated by the reason for the joke
            "noiseHigh"
            "attentionLow"
    Returns: response - [string] Return the joke
    """

    # define the query depending on the type
    if jokeType == "noiseHigh":
        query = f"come up with a short, funny joke to retain the concentration of a noisy classroom"
    elif jokeType == "attentionLow":
        query = f"come up with a short, funny joke to entertain a classrom of students"
    else:
        query = f"come up with a short, funny joke for me to tell my students"

    # get response to query using selelcted model
    response = chat_model.getResponse(query)
    
    return response
