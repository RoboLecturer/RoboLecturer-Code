# script for the generation of a lecture script based on recieved lecture slides

# import packages
from pkg.chat import chat_model
from tqdm import tqdm
############################################################ 

# TODO: add CURIE GPT and see the speed diff and content diff

# create a dynamic prompt that changes the lecture delivary style... Cos at the moment it is just too excited all the fucking time 
############################################################

def genScript(inputText, slideNum):
    """ Function to generate lecture script for each bullet-point in a presentation slide page
    Args:   inputText - [array][string] contains lines from slide
            slideNum - [int] the current slide number to track when we are on the title page
    Returns: script - [string] contains the lecture script for the slide
    """
    # script = []
    script = ""

    # if the incoming slide is the title slide
    if slideNum == 0:
        # join title slide lines together, seperated by a commar 
        inputText = ", ".join(inputText)
        query = f"introudce the following presentation title page in the style of an interesting lecturer: {inputText}"
        # get response to query using selelcted model
        response = chat_model.getResponse(query)
        
        return response

    # if incoming slide is the table of contents
    elif inputText[0].lower() == "table of contents":
        inputText = inputText[1:]
        # create a single string input with contents seperated by a new line
        inputText = "\n".join(inputText)

        query = f"very shortly introduce each of the following points, that are seperated by a new line, which are to be the topics covered in today's lecture, in the style of a lecturer: {inputText}"

        # get response to query using selelcted model
        response = chat_model.getResponse(query)
        
        return response
    
    else:
        for line in tqdm(inputText):
            # summarise the bullet point in a few sentences
            query = f"explain the following point in a single paragraph in the style of an interesting lecturer: {line}"

            # get response to query using selelcted model
            response = chat_model.getResponse(query)        

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

def createScript(Text, slide_num):
    """Main function to create a lecture script from lecture slides
    Args: Text - [string] input slide text
    Returns: Script - [string] lecture script
    """
    inputText = getText(Text)
    script = genScript(inputText, slide_num) 
    return script

