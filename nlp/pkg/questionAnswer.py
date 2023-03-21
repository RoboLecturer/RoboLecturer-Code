# This script recieves questions, and generates answers #
# use simple GPT method for MVP 

# import packages
from pkg.chat import chat_model

# function for generation the answer to questions based on the question
def answerGen(question, context, title):
    """This function generates the answer to a proposed question
    Args: question - [string] question recieved from Speech Processing Module
    Returns: response - [string] response to question generated by a GPT model
    """
    query = f"Answer this question - {question} - that relates to this context - {context} - and this topic - {title} -  in the style of a lecturer."
    # query = f"Answer the following question in a short paragraph in the style of an interesting lecturer: {question} - The answer must be in the context of this extract from a lecture script of title. {title}: {context} "

    # get response to query using selelcted model
    response = chat_model.getResponse(query)
    
    return response
