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
        query = f"grab the attention of noisy students by telling a funny joke to retain their concentration"
    elif jokeType == "attentionLow":
        query = f"grab the attention of uninterested students by telling a funny joke to entertain the classroom"
    else:
        query = f"Tell a group of students a funny joke"

    # get response to query using selelcted model
    response = chat_model.getResponse(query)
    
    return response
