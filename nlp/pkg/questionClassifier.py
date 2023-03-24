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
import openai


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
    threshold = 0.6

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
            elif sub_class == "finished":
                main_class = "finished"
            else:
                main_class = "related"

    # Classify the question based on the maximum similarity
    if main_class == "finished":
        return main_class, sub_class 

    if max_sim < threshold:
        main_class = "non-related"
        sub_class = ""
        return main_class, sub_class
    else:
        return main_class, sub_class
    
def is_coherent(query,topic):
    """call the openai chatGPT api
    @params: query [string]
    @reutrns: response [string]
    """
    openai.api_key = "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"
    # create completion
    completions = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
        messages=[
                    {"role": "system", "content": f""""
            you are classifying whether questions or statements make sense or not in the context of the situation.
            Respond only with the exact word "coherent" if the question or statement makes sense in context and with the word "incoherent"
            if the question or statement does not make sense in the context.
           
            context: you just gave a lecture on {topic} and are asking if people have questions.
           
            Coherent statement include statements or questions that make sense, but also operational requests
            such as speak louder, speak slower, or change the slide are coherent. Also if students use vocal disfluences such as
            "like", "umm", "ah" etc. in a statement but it is still makes sense semantically, classify it as coherent.
           
            """},
            {"role": "assistant", "content": "Do you have any questions>"},
                    {"role": "user", "content":"how are stars made?"},
            {"role": "assistant", "content":"coherent"},
            {"role": "user", "content":"what does this"},
            {"role": "assistant", "content":"incoherent"},
            {"role": "user", "content":f"{query}"},
                ],
          temperature = 0,
          max_tokens = 5
        )
    response = completions['choices'][0]['message']['content']
    return not "incoherent" in response.lower()