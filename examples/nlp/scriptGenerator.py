# script for the generation of a lecture script based on recieved lecture slides

# import packages
import openai
from tqdm import tqdm
############################################################ 
# TODO: create code to deal with the title 
# for the title page... take all the text and use a good prompt to get a simple explination return
# TODO: add CURIE GPT and see the speed diff and content diff

# create a dynamic prompt that changes the lecture delivary style... Cos at the moment it is just too excited all the fucking time 
############################################################

def genScript(inputText):
    """ Function to generate lecture script for each bullet-point in a presentation slide page
    Args: inputText - [array][string] contains lines from slide
    Returns: script - [array][string] contains the lecture script for the slide
    """
    # script = []
    script = ""

    # set-up the API key
    openai.api_key = "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"

    # Is the incoming slide the table of contents?
    if inputText[0].lower() == "table of contents":
        inputText = inputText[1:]
        # create a single string input with contents seperated by a new line
        inputText = "\n".join(inputText)
        query = f"shortly introduce each of the following points, that are seperated by a new line, as an introduction to a lecture with these points, in the style of an excited lecturer: {inputText}"
        # completions
        completions = openai.Completion.create(
            engine="text-davinci-003",
            prompt=query,
            max_tokens=1024,
            n=1, # generate a single completion
            temperature=0.2, # keeps responses narrow
        )
        response = completions.choices[0]["text"]
        return response 
    else:
        for line in tqdm(inputText):
            # summarise the bullet point in a few sentences
            query = f"explain the following point in a single paragraph in the style of an excited lecturer: {line}"
            # completions
            completions = openai.Completion.create(
                engine="text-davinci-003",
                prompt=query,
                max_tokens=1024,
                n=1,
                temperature=0.2,
            )
            response = completions.choices[0]["text"]
            # append the output if we need an array of string
            # script.append(response)
            # concatenate the output if we need a single string
            script = "\n".join([script,response])

    # return the array of responses 
    return script 

def getText(slideText):
    """Fucntion to extract the input text from the stream of presentation text published from web
    Arg: slideText - [string] presentation text string
    Reutrns: inputText - [array][string] array of the seperate presentaiton lines (for a single slide)
    """
    inputText = slideText.split("\n")
    return inputText

def createScript(Text):
    """Main function to create a lecture script from lecture slides
    Args: Text - [string] input slide text
    Returns: Script - [string] lecture script
    """
    inputText = getText(Text)
    script = genScript(inputText) 
    return script

