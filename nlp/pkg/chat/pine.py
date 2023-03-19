# PineCone handling

import pinecone 
import time
import uuid
from pkg.chat import chat_model
# import time

def init_pinecone():
    pinecone.init(
		api_key = "8973578e-4fed-4fff-8a8d-5eaf1d2d4032",
		environment = "us-central1-gcp"
	)
    indexName = "pepper-memory"
    if indexName not in pinecone.list_indexes():
        pinecone.create_index(
            indexName, 
            dimension=1536, # length of an ada vector embedding
            metric='cosine',
            metadata_config={'indexed': ['channel_id', 'published']}
        )

    vdb = pinecone.Index(indexName)
    return vdb

def populatePinecone(contents, nameSpace, metadata, vdb):
    """add elements to the pinecone database as vector embeddings
    @params:   
        content: [string] - payload to embed 
        vdb: object - the pinecone database
        nameSpace: "string" - namespace of the database
        metadata: contents metadata for storage in pinecone for filtering (content depends on the namepsace)
    """
    # set ids
    ids_batch = [x['id'] for x in metadata]
    embeds = chat_model.getEmbedding(contents) # returns a list of embeddings

    to_upsert = list(zip(ids_batch, embeds, metadata))
    # upsert to pinecone
    vdb.upsert(vectors=to_upsert, namespace=nameSpace)


def queryPinecone(query, vdb, namespace):
    """search for relevant strings in pinecone database
    @params: 
        query: [string] - query asked by the user
        vdb: database object
        namespace: [string] - namespace for the query
    @returns:
        contexts: list|string - list of the response strings"""
    # we need make sure that the respones are not too long
    xq = chat_model.getEmbedding(query)
    
    if namespace == "script":
        response = vdb.query(
            xq,
            top_k = 5, # take the 5 top matches (if 5 exist)
            include_metadata=True, # return the metadata, as this is where the payload will be
            namespace=namespace
        )
        contexts = [
            x['metadata']['text'] for x in response['matches']
        ]
        return contexts
    # process the contexts
    elif namespace == "conversation":
        response = vdb.query(
            xq,
            top_k=10,
            include_metadata = True,
            namespace=namespace,
            # filter conversation results to any that oocured in the last 3 mintues
            filter={
                "time": {'$gt': time.time() - 180}
            }
        )

        qContexts = [
            x['metadata']['question'] for x in response['matches']
        ]
        aContexts = [
            x['metadata']['answer']  for x in response['matches']
        ]
        context = list()
        for i in range(len(qContexts)):
            context.append(
                {'role': 'user', 'contents': qContexts[i]},
                {'role': 'user', 'contents': aContexts[i]}
                )

        return context

def createSlideMetadata(slideNo, script, title):
    """create the pinecone metadata for lecture scripts
    @params:
        slideNo: int - number of the slide
        script: string - lecture script content
        title: string - title of the slide
    @returns:
        metadata: {dict} - the slide metadata with unique id
    """

    metadata = {
        "slideNo": slideNo, 
        "title": title,
        "text": script,
        "id": str(uuid.uuid4())
    }

    return metadata

def createConvoMetadata(title, question, answer):
    """create the pinecone metadata for conversations
    @params:
        title: string - title of the slide
        question: string - question posed by used
        answer: string - response given by assistant
    @returns:
        metadata: {dict} - the conversation metadata with unique id and timestamp
    """

    metadata = {
        "title": title, 
        "question": question,
        "answer": answer, 
        "time": time.time(),
        "id": str(uuid.uuid4())
    }

    return metadata 