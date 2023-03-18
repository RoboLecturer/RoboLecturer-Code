# PineCone handling

import pinecone 
from pkg.chat import chat_model
# import time

def init_pinecone():
    pinecone.init(
		api_key = "",
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

def populatePinecone(contents, vdb):
    """add elements to the pinecone database as vector embeddings
    @params:   
        content: {dict}|"text"|"ids"|"title"|"slideNo" - text and its id
        vdb: object - the pinecone database
    """
    ids_batch = [x['ids'] for x in contents]
    embeds = chat_model.getEmbedding(contents) # returns a list of embeddings
    # clean up metadata
    contents = [{
        "text": x["text"],
        "ids": x["ids"],
        "title": x["title"],
        "slideNo": x["title"]
    } for x in contents]

    to_upsert = list(zip(ids_batch, embeds, contents))
    # upsert to pinecone
    vdb.upsert(vectors=to_upsert)


def queryPinecone(query, vdb):
    """search for relevant strings in pinecone database"""
    # we need make sure that the respones are not too long
    xq = chat_model.getEmbedding(query)
    # retrieve from pinecone
    response = vdb.query(
        xq,
        top_k=5,
        include_metadata=True
        )
    contexts = [
        x['metadata']['text'] for x in response['matches']
    ]
    return contexts

