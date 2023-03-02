# This script produces the content class (1 per slide) and the descriptive keywords associate with the slide content #

# import packages
import openai

# function to create the descriptions
def genKeywords(content):
    """This function creates a list of keywords from the contents of a lecture slide
    Args:
            content - [string] the contents of a lecture slide
    Returns: 
            keywords - [list][string] array of keywords
    """

    # set up API key
    openai.api_key = "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"
    # create query
    query = f"give me a list of 5 keywords associated with the following text: {content}"
    # create completion
    completions = openai.Completion.create(
        engine="text-davinci-003",
        prompt = query,
        max_tokens=1024,
        n=1,
        temperature=0.2,
    )
    response = completions.choices[0]["text"]
    # reformat for use
    keywords = createKeywords(response)
    return keywords

# reformat response
def createKeywords(response):
    """this function reformats the davinci keyword response into a usable list
    Args:  
            response - [string] response from davinci with keywords
    Returns:
            keywords - [list][string] list containing each keyword alone
    """
    # response from davinci is seperate by new lines
    list_from_reponse = response.split("\n")
    keywords = []
    for item in list_from_reponse:
        # remove empty elements created by \n\n
        if item == "":
            continue
        else:
            # remove the enumeration from the front of each keyword
            # [3:] because list will never be longer than 5
            # e.g - 1. Water ..... 5. Atmosphere
            keywords.append(item[3:])
    return keywords        

# get title and script content
def processText(slide, script):
    """This function extracts the slide title and the script content for use in keyword creation function
    Args:   slide - [string] contents of a slide
            script - [string] lecture script content as a single string
    Returns: 
        title - [string] title of the current slide
        content - [string] content of the current slide script
    """
    # split slide content into seperate lines
    lines_of_slide = slide.split("\n")
    # title is the first line
    title = lines_of_slide[0]
    content = script
    return title, content 

# create the description and keywords
def createDescription(slide,script,class_description):
    """ This function creates content type and keywords for a slide
    Args:
        slide - [string] content of a single slide
        script - [string] lecture script content
    Returns:
        class_description - [dict] { [string]: [[list][string]] } dictionary containing the class and corresponding keyworkds
    """
    # process text
    title, content = processText(slide, script)
    keywords = genKeywords(content)
    # add element to dictionary
    class_description[f"{title}"] = keywords

    return class_description