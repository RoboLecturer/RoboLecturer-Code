# Script for the classification of question type into the following categories
# Pertinant / non-pertinant / operational
# Sub-Class

#############################################################################
# TODO: Return Class and Sub-Class --->> DONE
# TODO: Create the descriptions using Davinci --->> DONE
# TODO: Wrap for use in main script --->> DONE
# TODO: wrap class description for function use --->> DONE
############################################################################# 

# import and installations
# pip install sentence_transformers
from sentence_transformers import SentenceTransformer, util


# Define a function to classify questions
def classify_question(question, class_descriptions):
    """Function to classify questions into their class type and sub-class type
    Args: 
            question - [string] incoming question for classification
            class_descriptions - [dict] { [string]: [[list][string]] } dictionary containing the class and corresponding keyworkds
    Returns: 
            main_type - [string] main class output (relevnat / non-relevant / operations)
            sub_type - [string] sub-class output (specific operational request, specific slide title)
    """

    # Load a pre-trained sentence transformer model
    model = SentenceTransformer('distilbert-base-nli-stsb-mean-tokens')

    # Generate embeddings for the question
    question_embedding = model.encode(question)

    # Define the classification threshold
    threshold = 0.4

    # define operational keys for later main_class checking
    operationalKeys = ["increase speech speed",
        "decrease speech speed",
        "increase speech volume",
        "decrease speech volume",
        "go to previous slide",
        "go to next slide",
        "go to specific slide number"]
    max_sim = -1
    main_class = "non-related"
    sub_class = ""
    # Calculate the similarity between the question embedding and each class description
    for class_name, class_desc in class_descriptions.items():
        sim = util.pytorch_cos_sim(question_embedding, model.encode(class_desc)).max().item()
        if sim > max_sim:
            max_sim = sim
            sub_class = class_name
            if sub_class in operationalKeys:
                main_class = "operational"
            else:
                main_class = "related"

    # Classify the question based on the maximum similarity
    if max_sim < threshold:
        main_class = "non-related"
        sub_class = ""
        return main_class, sub_class
    else:
        return main_class, sub_class
    
