# NLP Documentation
The three overall functionalities are:
- **```scriptGenerator```**: Generate lecture scripts based on slide content or table of contents
- **```questionClassifier```**: Classify the question into its relevant main and sub stypes. 
- **```questionAnswer```**: Answer a questions using engineered prompt and chatGPT completion. 

# SET-UP
First time users must update the import path to use their Openai and Pinecone API keys
```
import sys
sys.path.append('/your/path/here/')
from <file> import <api_key>
```


## Contents
- [Openai API](#openai)
- [Pinecone Database](#pinecone)
- [Prompt Engineering](#prompt-engineering) 
    - [Conversation Class](#conversation-class)
    - [Salinence](#salience)
    - [Anticipation](#anticipation)
    - [Context](#context)
- [Question Classification](#question-classification)
- [Completions](#completions)
- [Additional Functions](#additional-functions)
- [NLP DUMMY](#NLP-Dummy)

## Openai
For each creation, the openai API is used to query gpt-3.5-turbo using an engineered prompt. This returns usable lecture scripts, question answers and other useful text and can be extracted.

#### gpt-3.5-turbo
```
Import Openai

openai.api_key = <api_key>
# create completion
completions = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
                {"role": "system", "content": f"{query}"},
            ]
    )
response = completions['choices'][0]['message']['content']
```

#### ADA
The openai API is also used to create vector embeddings for queries to the pinecone database, to be introduced in a later section. This uses the ADA package. 

```
response = openai.Embedding.create(
            input = text,
            engine = 'text-embedding-ada-002'
        )
embeds = [record['embedding'] for record in response['data']]
```

## Pinecone
The pinecone vectorised database is used to store and retrieve relevant lecture information using cosine semantic similarity. Lecture scripts, textbooks and converstion histories and stored in seperate `namespaces` and can be queried over to get information relevant to the incoming user question. 

- **```init_pinecone()```**: Set up the pinecone database using the project name and key

- **```populatePinecone(contents, nameSpace, metadata, vdb)```**: Upsert data into the pinecone data base.
    - **params** (*String*) : name of the `namespace` that you want to upsert to
    ```nameSpace```
    - **params** (*Dict*) : metadata containing original data, slide number and id, along with other information as key-value pairs. 
    ```metadata```
    - **params** (*Vector*) : ada produced vector embeddings of the data to be upserted
    ```contents```
    - **params** (*pinecone-object*) : the pinecone database object
    ```vdb```

- **```queryPinecone(query, vdb, namespace, title)```**: Query over a specific `namespace` in the pinecone database. 
    - **params** (*String*) : name of the `namespace` that you want to upsert to
    ```nameSpace```
    - **params** (*String*) : used is a specific slide script is required
    ```title```
    - **params** (*Vector*) : ada produced vector embeddings of the query
    ```query```
    - **params** (*pinecone-object*) : the pinecone database object
    ```vdb```

- **```KnowledgeBaseToPinecone(url,topic)```**: Upsert a textbok to pinecone
    - **params** (*String*) : the url to the textbook pdf or text. 
    ```url```
    - **params** (*String*) : Name of the lecture topic
    ```topic```


## Prompt Engineering
Each prompt contains a number of aspects that are concatentated to form the query:
- prompt wrapper
    - Personal Context and goals
    - Conversation salience and anticipation
    - lecture history 
    - Aditiional context 

### Conversation Class
This class if used to store the methods and contents used when creating the prompt. Each prompt is stored as a *list* with 3 elements; personal context and conversation salience, lecture history and additional context. 

```
class convHistory:
	def __init__(self, context):
		self.context = context
		self.history = list()
		self.salience = ""
		self.anticipation = ""

		self.history.append({'role': 'system', 'content': self.context}) 
		self.history.append({'role': 'assistant', 'content': ""}) 
		self.history.append({'role': 'assistant', 'content': ""}) 
```
- **```__init__(self, context)```**: initialise the class with the context and prompt list elements
    - **params** (*String*) :  the personal context of Robo Lecturer. 
    ```context```

The following subsections encapsulate the methods used in the conversation class to build the prompt

### Salience

- **```getSalience(self, convoContext)```**: generate the salience of previous conversational exchanges (where these exchanges have been pulled from pinecone by time stamp). 
    - **params** (*string*): previous conversations as a sinlge string 
    ```convoContext```
    - **returns** (*string*): Summarised conversation as a single string

### Anticipation

- **```getAnticipation(self)```**: generate the anticipation of the users needs using the current conversation list, which includes: personal context and goals, conversation salience, and relevant lecture scripts. 

### Context

- **```updateSlideContext(self, script, context)```**: update the content context for the prompt *list*. Add the script to the second element and the additional context to the third element. 
    - **params** (*String*) : the contents of the lecture script that relates to the query. Pulled from pinecone. 
    ```script```
    - **params** (*String*) : Additional context that has been gathered from the pinecone database by using semantic similarity between the query and any relevant textbook text. 
    ```context```


## Question Classification
For each user question. The query is classified into its relevant type; relevant, non-relevant or operational. The query is also tested for coherance. 

- **```is_coherent(query,topic)```**: check to see if the question is coherent. If not, maybe the speech module failed to interperate the question correctly.
    - **params** (*string*) : The questions in a single string
    ```query```
    - **params** (*string*) : The title of the lecture as a single string
    ```topic```

- **```classify_question(question, class_descriptions)```**: Classify the question using the `sentance transformers` stack. This checks for semantic similarity between the question and the class descriptions dictionary, which contains the lecture slide titles and keyphrases as key-value pairs. 
    - **params** (*String*) : question as a single string
    ```question```
    - **params** (*Dict*) : Dictionary of the lecture slide titles as `keys` and key-phrases (*String*) as `values`. 
    ```class_descriptions```

- **```initDict()```**: Initialise the class description dictionary with the operational key-values pairs. (examples include `raise volume`)

- **```createDescription(slide,script,class_description)```**: Create key-value pairs to add to the class descriptions dictionary using the slide title and its lecture content. 
    - **params** (*String*) : title of the lecture slide
    ```slide```
    - **parmas** (*String*) : content of the lecture script as a single string
    ```script```
    - **params** (*Dict*) : Current class description dictionary to be appended to.


## Completions
To get the responses to any prompt, an openai completion must be created using the following functions. 

- **```chatGPT(query)```**: Get the response from the `gpt-3.5-turbo` model. 
    - **params** (*String*) : The engineered prompt as a single string. (prompt *list* that has been flattened)

- **```daVinci(query)```**: Get the response from the `davinci` model. This is a backup incase the gpt-3.5-turbo model fails for some reason. 
    - **params** (*String*) : The engineered prompt as a single string. (prompt *list* that has been flattened)


## Additional Functions
Here are additional functions that are not essential to the functionality of the Robo-Lec

- **```genJoke(jokeType)```**: Generate a joke for when the class gets too loud to in-attentive
    - **params** (*String*) : What kind of joke do we want?
    ```jokeType```

- **```quizGen(script)```**: Generate a quiz for each lecture slide for using throughout the lecture
    - **params** (*String*) : the lecture script for a single slide
    ```script```

- **```generate_presentation(topics)```**: Generate the lecture scripts and slides from a table of contents or list of topics
    - **params** (*Array*) : The list of the topics that are to be taugh in the lecture. 


