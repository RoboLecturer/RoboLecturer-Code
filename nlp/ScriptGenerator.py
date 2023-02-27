# Script for generating the lecture script from pre-made lecture slides #

# import packages
import openai

# recieve the incoming slide page as text file
# incomingSlide = input("Enter slide here:")
incomingSlide = "This is an incoming slide page.\nThis is the second bullet point.\nThis is the thrid."

# sort into seperate points (split by new line)
inputText = incomingSlide.split("\n")

# set-up the API key
openai.api_key = "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"


def genScript(inputText):
    """ Function to generate lecture script for each bullet-point in a presentation slide page
    Args: inputText - [string] contains lines from slide
    Returns: script - [array][string] contains the lecture script for the slide
    """
    # script = []
    script = ""
    for line in inputText:
        # summarise the bullet point in a few sentences
        query = f"explain the following point in a single paragraph in the style of an excited lecturer: {line}"
        # completions
        completions = openai.Completion.create(
            engine="text-davinci-003",
            prompt=query,
            max_tokens=1024,
            n=1,
            stop=None,
            temperature=0.2,
        )
        response = completions.choices[0]["text"]
        # append the output if we need an array of string
        # script.append(response)
        # concatenate the output if we need a single string
        script = "\n".join([script,response])

    # return the array of responses 
    return script 


script = genScript(inputText)
print(script)
