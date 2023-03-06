# script for the generation of a lecture script based on recieved lecture slides

# import packages
import openai
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

    # set-up the API key
    openai.api_key = "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"

    # if the incoming slide is the title slide
    if slideNum == 0:
        # join title slide lines together, seperated by a commar 
        inputText = ", ".join(inputText)
        query = f"introudce the following presentation title page in the style of an excited lecturer: {inputText}"
        # create completion
        completions = openai.ChatCompletion.create(
		    model="gpt-3.5-turbo",
		    messages=[
				    {"role": "system", "content": f"{query}"},
			    ]
	    )
        response = completions['choices'][0]['message']['content']
        return response

    # if incoming slide is the table of contents
    elif inputText[0].lower() == "table of contents":
        inputText = inputText[1:]
        # create a single string input with contents seperated by a new line
        inputText = "\n".join(inputText)
        query = f"shortly introduce each of the following points, that are seperated by a new line, as a table of contents to a lecture with these points, in the style of an excited lecturer: {inputText}"
        # completions
        completions = openai.ChatCompletion.create(
		    model="gpt-3.5-turbo",
		    messages=[
				    {"role": "system", "content": f"{query}"},
			    ]
	    )
        response = completions['choices'][0]['message']['content']
        return response 
    
    else:
        for line in tqdm(inputText):
            # summarise the bullet point in a few sentences
            query = f"explain the following point in a single paragraph in the style of an excited lecturer: {line}"
            # completions
            completions = openai.ChatCompletion.create(
		        model="gpt-3.5-turbo",
		        messages=[
				        {"role": "system", "content": f"{query}"},
			        ]
	        )
            response = completions['choices'][0]['message']['content']
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

def createScript(Text, slide_num):
    """Main function to create a lecture script from lecture slides
    Args: Text - [string] input slide text
    Returns: Script - [string] lecture script
    """
    inputText = getText(Text, slide_num)
    script = genScript(inputText) 
    return script

