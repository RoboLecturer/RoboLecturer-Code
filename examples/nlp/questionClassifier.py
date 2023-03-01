# Script for the classification of question type into the following categories
# Pertinant / non-pertinant / operational
# Sub-Class

#############################################################################
# TODO: Return Class and Sub-Class
# TODO: Create the descriptions using Davinci
# TODO: Wrap for use in main script
# TODO: wrap class description for function use
############################################################################# 

# import and installations
# pip install sentence_transformers
from sentence_transformers import SentenceTransformer, util

# Define the class descriptions
class_descriptions = {
    # operational classes
    "increase speech speed": ["speed up", "faster", "quickly"],
    "decrease speech speed": ["slow down", "slower", "more slowly"],
    "increase speech volume": ["louder", "increase volume", "raise voice","speak up"],
    "decrease speech volume": ["softer", "lower volume", "quieter"],
    "go to previous slide": ["previous slide", "go back"],
    "go to next slide": ["next slide", "advance slide"],
    "go to specific slide number": ["go to slide"],
    # lecture content classes
    "astronomy": ["astronomy","stars", "planets", "universe"],
    "photosynthesis": ["photosynthesis","plants", "sunlight", "chlorophyll"],
    "plant cells": ["cell wall", "chloroplast", "vacuole"],
    "animal cells": ["mitochondria", "nucleus", "cytoplasm"]
}

# Define a function to classify questions
def classify_question(question):
    """Function to classify questions into their class type and sub-class type
    Args: question - [string] incoming question for classification
    Returns: chosen_class - [string] Class type string
    """

    # Load a pre-trained sentence transformer model
    model = SentenceTransformer('distilbert-base-nli-stsb-mean-tokens')

    # Generate embeddings for the question
    question_embedding = model.encode(question)

    # Define the classification threshold
    threshold = 0.4

    # Calculate the similarity between the question embedding and each class description
    max_sim = -1
    chosen_class = "non-lecture related question"
    
    for class_name, class_desc in class_descriptions.items():
        sim = util.pytorch_cos_sim(question_embedding, model.encode(class_desc)).max().item()
        if sim > max_sim:
            max_sim = sim
            chosen_class = class_name

    # Classify the question based on the maximum similarity
    if max_sim < threshold:
        return "non-lecture related question"
    else:
        return chosen_class
    
