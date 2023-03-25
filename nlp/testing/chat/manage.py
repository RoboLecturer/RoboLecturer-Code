
from time import time
import datetime
import json
import os

def open_file(filepath):
    with open(filepath, 'r', encoding='utf-8') as infile:
        return infile.read()


def save_file(filepath, content):
    with open(filepath, 'w', encoding='utf-8') as outfile:
        outfile.write(content)


def load_json(filepath):
    with open(filepath, 'r', encoding='utf-8') as infile:
        return json.load(infile)

def timestamp_to_datetime(unix_time):
    return datetime.datetime.fromtimestamp(unix_time).strftime("%A, %B %d, %Y at %I:%M%p %Z")

def save_json(filepath, payload):
    with open(filepath, 'w', encoding='utf-8') as outfile:
        json.dump(payload, outfile, ensure_ascii=False, sort_keys=True, indent=2)


# def gpt3_embedding(content, engine='text-embedding-ada-002'):
#     content = content.encode(encoding='ASCII', error='ignore').decode()
#     response = openai.Embedding.create(input=content,engine=engine)
#     vector = response['data'][0]['embedding']
#     return vector

def flatten_convo(conversation):
    convo=""
    for i in conversation:
        convo += '%s: %s\m' % (i['role'].upper(), i['content'])
    return convo.strip()

    # filename = '%s_chat.txt' % time()
    # if not os.path.exists('gpt3_logs'):
    #     os.makedirs('gpt3_logs')
    # save_file('gpt3_logs/%s' % filename, str(query) + '\n\n==========\n\n' + response)
