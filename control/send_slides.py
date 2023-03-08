import PepperAPI
from PepperAPI import Info

PepperAPI.init("master")
list_of_slides_text = [
	"Table of Contents\nOur Solar System\nHow stars are born\nThe Universe",
	"Our solar system contains planets and stars\nThe earth is the only planet that sustains life",
	"How are stars born"
]

# Send text for all slides to Web one by one
number_of_slides = len(list_of_slides_text)
Info.Send("NumSlides", {"value": number_of_slides})	# tells NLP how many slides to receive
for slide in list_of_slides_text:
	Info.Send("Slides", {"text": slide})
