# this script is for creating a quiz based on a slide, the answers (true and false) and then a short explination of the answer 

# imports
import openai

# function for generating the quiz
def quizGen(script):
    """generate a 5 question quiz using the contents of a lecture slide script. Each question should have 5 answers, one of which is correct. Export the file in the json format.
    Args:   script - [string] contents of lecture script
    Returns:    quiz - [string] quiz with all relevant information in the json format, but as a single string
    """
    openai.api_key = "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"

    response = openai.ChatCompletion.create(
        model = "gpt-3.5-turbo",
        message = [
            {"role": "system", "content": f"Generate a quiz with 5 questions, each with 5 choices based on the following extract from a lecture: {script}. The quiz must be exported in the json format using the following fields: quiz_name, question_count, questions, prompt, answer, options"}
        ]
    )
    answer = response["choices"][0]["message"]["content"]

    return answer

